#!/usr/bin/env python3
"""Single-window Gaussian correlation mechanism; never a live state estimator."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from xq_autonomy.correlated_update import correlated_gaussian_update


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def wilson(successes, total, z=1.959963984540054):
    p = successes / total
    den = 1 + z*z / total
    centre = (p + z*z / (2*total)) / den
    radius = z * math.sqrt(p*(1-p)/total + z*z/(4*total*total)) / den
    return [centre-radius, centre+radius]


def study(protocol, output):
    if output.exists():
        raise ValueError('study output must be new')
    output.mkdir(parents=True)
    cfg = protocol['synthetic_study']
    limits = cfg['checks']
    n, m = cfg['state_dimension'], 2
    indices = cfg['observed_state_indices']
    if n != 19 or indices != [7, 8] or cfg['observed_prior_sigma_mps'] != cfg['measurement_noise_sigma_mps']:
        raise ValueError('this analytic control requires the frozen symmetric 19-state fixture')
    sigma = cfg['observed_prior_sigma_mps']
    prior = np.zeros(n)
    prior[indices] = cfg['prior_velocity_xy_mps']
    covariance = np.diag([1e-4]*3 + [0.01]*3 + [1e-5] + [sigma*sigma]*3 + [1e-6]*3 + [1e-4]*6)
    h = np.zeros((m, n))
    h[np.arange(m), indices] = 1
    noise_covariance = np.eye(m) * sigma*sigma
    direction = np.asarray(cfg['prior_velocity_xy_mps'], dtype=float)
    direction /= np.linalg.norm(direction)
    samples, cases, checks = {}, [], {}
    # Missing C is an unavailable interface, never evidence of independence.
    try:
        correlated_gaussian_update(prior, covariance, h @ prior, h, noise_covariance, None)
    except ValueError:
        checks['unknown_cross_covariance_rejected'] = True
    else:
        checks['unknown_cross_covariance_rejected'] = False
    for case_index, rho in enumerate(cfg['state_noise_correlations']):
        cross = np.zeros((n, m))
        cross[indices, :] = np.eye(m) * rho * sigma*sigma
        full = correlated_gaussian_update(prior, covariance, h @ prior, h, noise_covariance, cross)
        naive = correlated_gaussian_update(prior, covariance, h @ prior, h, noise_covariance, np.zeros_like(cross))
        gains = {'keep_C': np.asarray(full['gain']), 'force_C_zero': np.asarray(naive['gain'])}
        post = {'keep_C': np.asarray(full['posterior_covariance']),
                'force_C_zero': np.asarray(naive['posterior_covariance'])}
        analytic = covariance.copy()
        analytic[np.ix_(indices, indices)] = np.eye(m) * sigma*sigma*(1-rho)/2
        checks[f'case{case_index}.analytic_posterior'] = bool(np.max(np.abs(post['keep_C']-analytic)) <= limits['maximum_analytic_posterior_covariance_error'])
        all_errors = {name: [] for name in gains}
        max_mean_difference = 0.0
        for seed in cfg['seeds']:
            rng = np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed, case_index])))
            latent = rng.standard_normal((cfg['draws_per_seed_and_case'], 6))
            g, a, b = latent[:, :2], latent[:, 2:4], latent[:, 4:]
            error = sigma * (math.sqrt(abs(rho))*g + math.sqrt(1-abs(rho))*a)
            noise = sigma * (np.sign(rho)*math.sqrt(abs(rho))*g + math.sqrt(1-abs(rho))*b)
            residual = error + noise
            key = f'case{case_index}_seed{seed}'
            samples[key+'_latent'] = latent
            samples[key+'_prior_error_xy'] = error
            samples[key+'_measurement_noise'] = noise
            samples[key+'_innovation'] = residual
            corrections = {name: residual @ gain[indices, :].T for name, gain in gains.items()}
            max_mean_difference = max(max_mean_difference, float(np.max(np.abs(corrections['keep_C']-corrections['force_C_zero']))))
            for name in gains:
                posterior_error = error - corrections[name]
                all_errors[name].append(posterior_error)
                samples[key+'_'+name+'_posterior_error_xy'] = posterior_error
        checks[f'case{case_index}.same_point_estimates'] = max_mean_difference <= limits['maximum_posterior_mean_method_difference']
        row = {'case_index': case_index, 'rho': rho, 'trials': cfg['draws_per_seed_and_case']*len(cfg['seeds']),
               'maximum_point_estimate_difference_mps': max_mean_difference, 'methods': {}}
        for name, parts in all_errors.items():
            errors = np.concatenate(parts)
            predicted = post[name][np.ix_(indices, indices)]
            empirical = np.cov(errors, rowvar=False, bias=True)
            variance = float(direction @ predicted @ direction)
            successes = int(np.count_nonzero(np.abs(errors @ direction) <= cfg['coverage']['normal_quantile']*math.sqrt(variance)))
            relative_error = float(np.linalg.norm(empirical-predicted) / np.linalg.norm(predicted))
            row['methods'][name] = {'covered_trials': successes, 'coverage': successes/len(errors),
                'wilson_95': wilson(successes, len(errors)), 'predicted_covariance_xy_mps2': predicted.tolist(),
                'empirical_covariance_xy_mps2': empirical.tolist(), 'predicted_directional_variance_mps2': variance,
                'empirical_directional_variance_mps2': float(direction @ empirical @ direction),
                'relative_covariance_frobenius_error': relative_error}
        correct, wrong = row['methods']['keep_C'], row['methods']['force_C_zero']
        checks[f'case{case_index}.correct_coverage'] = limits['correct_coverage_interval'][0] <= correct['coverage'] <= limits['correct_coverage_interval'][1]
        checks[f'case{case_index}.correct_covariance'] = correct['relative_covariance_frobenius_error'] <= limits['maximum_relative_posterior_covariance_error']
        if rho < 0:
            checks[f'case{case_index}.negative_correlation_undercoverage'] = correct['coverage']-wrong['coverage'] >= limits['minimum_negative_correlation_coverage_gap']
        elif rho > 0:
            checks[f'case{case_index}.positive_correlation_overcoverage'] = wrong['coverage'] >= limits['minimum_positive_correlation_independence_coverage']
        else:
            checks[f'case{case_index}.zero_correlation_control'] = bool(np.max(np.abs(post['keep_C']-post['force_C_zero'])) <= limits['maximum_zero_correlation_method_difference'] and correct['covered_trials'] == wrong['covered_trials'])
        cases.append(row)
    np.savez_compressed(output/'samples.npz', **samples)
    index = {'path': str((output/'samples.npz').resolve()), 'bytes': (output/'samples.npz').stat().st_size,
             'sha256': digest(output/'samples.npz'), 'arrays': {key: {'shape': list(value.shape), 'dtype': str(value.dtype)} for key, value in samples.items()}}
    write(output/'raw-array-index.json', index)
    # Read every saved array, including unmodified latent draws. A file existing
    # on disk does not establish that it is a usable or complete result artifact.
    with np.load(output/'samples.npz', allow_pickle=False) as archive:
        checks['all_raw_arrays_read_back_identically'] = set(archive.files) == set(samples) and all(np.array_equal(archive[key], value) for key, value in samples.items())
    checks['sample_archive_unchanged_after_read'] = index['sha256'] == digest(output/'samples.npz')
    result = {'schema_version': 1, 'status': 'PASS' if all(checks.values()) else 'FAIL',
        'input_origin': cfg['input_origin'], 'checks': checks, 'cases': cases,
        'independent_trials': sum(row['trials'] for row in cases), 'raw_arrays': index,
        'scope': 'Known linear Gaussian signed-correlation mechanism only; not actual flow fusion, calibrated sensor error, statistical integrity or a new filter method'}
    write(output/'study-result.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = study(json.loads(args.protocol.read_text()), args.output)
    print(json.dumps({'status': result['status'], 'checks': len(result['checks']), 'independent_trials': result['independent_trials'], 'cases': result['cases']}))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
