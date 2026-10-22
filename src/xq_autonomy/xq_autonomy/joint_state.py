"""Explicit joint Gaussian bookkeeping for an OFFLINE LINEAR fixture.

The active vector has 19 arbitrary linear coordinates; coordinate 19 is one
persistent nuisance.  Clones copy active coordinates 0:6 deterministically.
These coordinates are NOT a physical attitude/pose manifold, calibrated range,
IMU propagation, FAST-LIVO state, sensor driver, or protection level.

All probabilities are conditional on the same accumulated information.  New
process/measurement noise must be explicitly declared independent of that
information and the complete joint state.  Shared errors belong in the joint
state and measurement matrix; unknown cross covariance is never replaced by
zero.  Time stamps are caller-defined discrete-model times in integer ns, with
no claim about ROS clocks, exposure times, synchronization, or real hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Integral, Real
from types import MappingProxyType

import numpy as np

from .correlated_update import correlated_gaussian_update


ACTIVE_DIMENSION = 19
NUISANCE_DIMENSION = 1
CLONE_DIMENSION = 6


def _numeric(value, shape, name):
    def contains_bool(item):
        if isinstance(item, (bool, np.bool_)):
            return True
        return isinstance(item, (list, tuple)) and any(contains_bool(v) for v in item)

    try:
        raw = np.asarray(value)
        if raw.dtype.kind not in "iuf" or contains_bool(value):
            raise ValueError
        parsed = raw.astype(np.float64)
        if parsed.shape != shape or not np.all(np.isfinite(parsed)):
            raise ValueError
        return parsed
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(name + " must have shape " + str(shape) + " and finite real entries") from exc


def _tolerance(value):
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, Real)
            or not math.isfinite(value) or not 0.0 < value <= 1.0e-9):
        raise ValueError("correlation_tolerance must be positive and at most 1e-9")
    return float(value)


def _stamp(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral) or value < 0:
        raise ValueError(name + " must be a nonnegative integer model timestamp in ns")
    return int(value)


def _covariance(value, dimension, name, tolerance):
    matrix = _numeric(value, (dimension, dimension), name)
    diagonal = np.diag(matrix)
    if np.any(diagonal < 0.0):
        raise ValueError(name + " has a negative variance")
    zero = diagonal == 0.0
    if np.any(matrix[zero, :] != 0.0) or np.any(matrix[:, zero] != 0.0):
        raise ValueError(name + " has nonzero cross covariance at a zero variance")
    scales = np.sqrt(np.where(zero, 1.0, diagonal))
    try:
        with np.errstate(over="raise", divide="raise", invalid="raise"):
            balanced = matrix / scales[:, None] / scales[None, :]
            asymmetry = float(np.max(np.abs(balanced - balanced.T)))
            if asymmetry > tolerance:
                raise ValueError(name + " is not symmetric in correlation coordinates")
            balanced = 0.5 * balanced + 0.5 * balanced.T
            minimum = float(np.linalg.eigvalsh(balanced)[0])
            if minimum < -tolerance:
                raise ValueError(name + " is not PSD in correlation coordinates")
    except (FloatingPointError, np.linalg.LinAlgError) as exc:
        raise ValueError(name + " cannot be evaluated without numerical repair") from exc
    # Remove only declared roundoff asymmetry, never eigenvalues or variances.
    return 0.5 * matrix + 0.5 * matrix.T, {
        "scaled_asymmetry_max": asymmetry,
        "scaled_minimum_eigenvalue": minimum,
        "correlation_tolerance": tolerance,
        "jitter_added": False,
        "eigenvalues_clipped": False,
    }


def _immutable(array):
    # A normal read-only owning ndarray can have WRITEABLE re-enabled.  A bytes
    # backing cannot; callers therefore cannot silently change retained moments.
    return np.frombuffer(array.tobytes(), dtype=np.float64).reshape(array.shape)


@dataclass(frozen=True)
class JointGaussianState:
    """Immutable moments created by create_joint_state and the operations below."""

    _mean: np.ndarray
    _covariance: np.ndarray
    timestamp_ns: int
    _clones: tuple
    correlation_tolerance: float
    _diagnostics: object

    def __post_init__(self):
        # Retain the same validation/immutability if a caller explicitly
        # constructs or dataclasses.replace()s a state for a model ablation.
        tolerance = _tolerance(self.correlation_tolerance)
        stamp = _stamp(self.timestamp_ns, "timestamp_ns")
        try:
            clones = tuple(tuple(item) for item in self._clones)
        except TypeError as exc:
            raise ValueError("clones must contain (clone_id, model_timestamp_ns) pairs") from exc
        names = set()
        for item in clones:
            if len(item) != 2 or not isinstance(item[0], str) or not item[0] or item[0] != item[0].strip():
                raise ValueError("clones must contain nonempty unique names and timestamps")
            if item[0] in names or _stamp(item[1], "clone timestamp_ns") > stamp:
                raise ValueError("clone names must be unique and timestamps cannot be in the future")
            names.add(item[0])
        dimension = 20 + 6 * len(clones)
        mean = _numeric(self._mean, (dimension,), "joint mean")
        covariance, _ = _covariance(self._covariance, dimension, "joint covariance", tolerance)
        try:
            diagnostics = MappingProxyType(dict(self._diagnostics))
        except (TypeError, ValueError) as exc:
            raise ValueError("diagnostics must be a mapping") from exc
        object.__setattr__(self, "_mean", _immutable(mean))
        object.__setattr__(self, "_covariance", _immutable(covariance))
        object.__setattr__(self, "_clones", clones)
        object.__setattr__(self, "timestamp_ns", stamp)
        object.__setattr__(self, "correlation_tolerance", tolerance)
        object.__setattr__(self, "_diagnostics", diagnostics)

    @property
    def mean(self):
        return self._mean

    @property
    def covariance(self):
        return self._covariance

    @property
    def dimension(self):
        return int(self._mean.size)

    @property
    def active_indices(self):
        return tuple(range(ACTIVE_DIMENSION))

    @property
    def nuisance_indices(self):
        return (ACTIVE_DIMENSION,)

    @property
    def clone_ids(self):
        return tuple(item[0] for item in self._clones)

    def clone_indices(self, clone_id):
        try:
            offset = self.clone_ids.index(clone_id)
        except ValueError as exc:
            raise ValueError("unknown clone_id: " + str(clone_id)) from exc
        start = ACTIVE_DIMENSION + NUISANCE_DIMENSION + CLONE_DIMENSION * offset
        return tuple(range(start, start + CLONE_DIMENSION))

    def clone_timestamp_ns(self, clone_id):
        self.clone_indices(clone_id)
        return self._clones[self.clone_ids.index(clone_id)][1]

    def to_dict(self):
        return {
            "mean": self.mean.tolist(),
            "covariance": self.covariance.tolist(),
            "dimension": self.dimension,
            "timestamp_ns": self.timestamp_ns,
            "active_indices": list(self.active_indices),
            "nuisance_indices": list(self.nuisance_indices),
            "clones": [{"clone_id": name, "timestamp_ns": stamp,
                        "indices": list(self.clone_indices(name))}
                       for name, stamp in self._clones],
            "diagnostics": dict(self._diagnostics),
            "scope": "offline_discrete_linear_joint_gaussian_fixture",
            "physical_rotation_model": False,
            "livo_integration": False,
            "hardware_validated": False,
            "protection_level_available": False,
        }


@dataclass(frozen=True)
class JointUpdateResult:
    state: JointGaussianState
    gain: np.ndarray
    innovation: np.ndarray
    innovation_covariance: np.ndarray
    diagnostics: object


def _state(mean, covariance, stamp, clones, tolerance, operation):
    dimension = ACTIVE_DIMENSION + NUISANCE_DIMENSION + CLONE_DIMENSION * len(clones)
    parsed_mean = _numeric(mean, (dimension,), "joint mean")
    parsed_covariance, diagnostics = _covariance(covariance, dimension, "joint covariance", tolerance)
    diagnostics["operation"] = operation
    return JointGaussianState(_immutable(parsed_mean), _immutable(parsed_covariance), stamp,
                              tuple(clones), tolerance, MappingProxyType(diagnostics))


def _require_state(state):
    if not isinstance(state, JointGaussianState):
        raise ValueError("state must be a JointGaussianState")


def create_joint_state(
    active_mean,
    active_covariance,
    nuisance_mean,
    nuisance_covariance,
    active_nuisance_cross_covariance,
    *,
    timestamp_ns,
    correlation_tolerance=1.0e-12,
):
    """Create active19+nuisance1, requiring the complete initial joint moments.

    An explicit zero (19,1) cross block is valid when justified by the caller.
    None/unknown cross or covariance is rejected.  Singular PSD is allowed.
    """
    tolerance = _tolerance(correlation_tolerance)
    stamp = _stamp(timestamp_ns, "timestamp_ns")
    if active_nuisance_cross_covariance is None:
        raise ValueError("active_nuisance_cross_covariance is required; unknown is not zero")
    mean = _numeric(active_mean, (19,), "active_mean")
    nuisance = _numeric(nuisance_mean, (1,), "nuisance_mean")
    p, _ = _covariance(active_covariance, 19, "active_covariance", tolerance)
    nuisance_p, _ = _covariance(nuisance_covariance, 1, "nuisance_covariance", tolerance)
    cross = _numeric(active_nuisance_cross_covariance, (19, 1), "active_nuisance_cross_covariance")
    return _state(np.concatenate((mean, nuisance)), np.block([[p, cross], [cross.T, nuisance_p]]),
                  stamp, (), tolerance, "create_joint_state")


def clone_pose(state, clone_id):
    """Append an exact copy of current active[0:6], including every cross block.

    No independent noise is added.  The expected singularity is retained.
    """
    _require_state(state)
    if not isinstance(clone_id, str) or not clone_id or clone_id != clone_id.strip():
        raise ValueError("clone_id must be a nonempty string without surrounding whitespace")
    if clone_id in state.clone_ids:
        raise ValueError("clone_id already exists")
    transform = np.vstack((np.eye(state.dimension), np.eye(state.dimension)[:6]))
    return _state(transform @ state.mean, transform @ state.covariance @ transform.T,
                  state.timestamp_ns, state._clones + ((clone_id, state.timestamp_ns),),
                  state.correlation_tolerance, "clone_pose")


def propagate_active(
    state,
    transition,
    process_covariance,
    offset,
    *,
    next_timestamp_ns,
    independent_process_noise,
):
    """Apply x_active_next = F x_active + offset + q to the COMPLETE state.

    Only active coordinates receive Q.  Nuisance/clones remain fixed random
    variables and retain updated cross covariances.  q must be independent of
    all prior variables and observations; unknown/shared q is unsupported.
    """
    _require_state(state)
    if independent_process_noise is not True:
        raise ValueError("independent_process_noise must be explicitly True; shared/unknown noise is unsupported")
    stamp = _stamp(next_timestamp_ns, "next_timestamp_ns")
    if stamp <= state.timestamp_ns:
        raise ValueError("next_timestamp_ns must strictly advance the model clock")
    f = _numeric(transition, (19, 19), "transition")
    q, _ = _covariance(process_covariance, 19, "process_covariance", state.correlation_tolerance)
    drift = _numeric(offset, (19,), "offset")
    transform = np.eye(state.dimension)
    transform[:19, :19] = f
    with np.errstate(over="ignore", invalid="ignore"):
        mean = transform @ state.mean
        mean[:19] += drift
        covariance = transform @ state.covariance @ transform.T
        covariance[:19, :19] += q
    return _state(mean, covariance, stamp, state._clones, state.correlation_tolerance,
                  "propagate_active_explicit_independent_process_noise")


def linear_update(
    state,
    measurement,
    measurement_matrix,
    measurement_noise_covariance,
    *,
    measurement_timestamp_ns,
    independent_measurement_noise,
):
    """Condition every retained block with z = H joint_state + independent v.

    H may include active, nuisance, and any retained clone.  New v must be
    conditionally independent of the entire joint state and previous noises.
    In particular, put persistent scale error in the nuisance column of H;
    do not repeatedly insert it as fresh white R.  All conditional means and
    cross blocks are updated, including clones and variables absent from H.
    """
    _require_state(state)
    if independent_measurement_noise is not True:
        raise ValueError("independent_measurement_noise must be explicitly True; shared/unknown noise is unsupported")
    stamp = _stamp(measurement_timestamp_ns, "measurement_timestamp_ns")
    if stamp != state.timestamp_ns:
        raise ValueError("measurement_timestamp_ns must equal the current model timestamp")
    raw_measurement = np.asarray(measurement)
    if raw_measurement.ndim != 1 or not raw_measurement.size:
        raise ValueError("measurement must be a nonempty finite real vector")
    m = int(raw_measurement.size)
    z = _numeric(measurement, (m,), "measurement")
    h = _numeric(measurement_matrix, (m, state.dimension), "measurement_matrix")
    r, _ = _covariance(measurement_noise_covariance, m, "measurement_noise_covariance",
                       state.correlation_tolerance)
    result = correlated_gaussian_update(
        state.mean, state.covariance, z, h, r, np.zeros((state.dimension, m)),
        correlation_tolerance=state.correlation_tolerance,
    )
    updated = _state(result["posterior_mean"], result["posterior_covariance"], state.timestamp_ns,
                     state._clones, state.correlation_tolerance,
                     "linear_update_explicit_independent_measurement_noise")
    diagnostics = dict(result["diagnostics"])
    diagnostics.update({"all_joint_blocks_updated": True,
                        "fresh_measurement_noise_independent": True,
                        "shared_errors_must_be_explicit_state_coordinates": True})
    return JointUpdateResult(updated, _immutable(np.asarray(result["gain"])),
                             _immutable(np.asarray(result["innovation"])),
                             _immutable(np.asarray(result["innovation_covariance"])),
                             MappingProxyType(diagnostics))


def drop_clone(state, clone_id):
    """Marginalize by retaining a principal submatrix; NEVER condition on clone.

    Removing memory of an old variable does not observe its value.  A Schur
    complement here would incorrectly reduce retained uncertainty.
    """
    _require_state(state)
    removed = set(state.clone_indices(clone_id))
    keep = tuple(i for i in range(state.dimension) if i not in removed)
    clones = tuple(item for item in state._clones if item[0] != clone_id)
    return _state(state.mean[list(keep)], state.covariance[np.ix_(keep, keep)], state.timestamp_ns,
                  clones, state.correlation_tolerance, "drop_clone_principal_marginal")
