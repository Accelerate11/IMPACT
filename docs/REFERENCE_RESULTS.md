# Reference results and acceptance / 参考结果与验收

[中文入口](../README.md) · [English](../README.en.md)

The checked-in reference is one recorded **Gazebo simulation** (about 83.753 s), not a hardware trajectory. Its reference tag is `server-sim-2026.10.22`, commit `5750e6e204b697349e7b08bc1aca5e0bee1d51a9`. Later documentation/license metadata corrections do not change the numerical estimator code. Record the commit used for each new run.

## Five methods

| CLI name | What changes | ATE RMS (mm) | Maximum error (mm) | Actual flow state/covariance updates |
|---|---|---:|---:|---:|
| `native` | Native 19-D LIVO; no auxiliary joint state | 7.368785 | 121.779016 | 0 |
| `joint_no_flow` | Joint/clone/plane machinery; pixel update disabled | 7.203131 | 117.431185 | 0 |
| `full` | Full declared joint correlation model | 6.644720 | 116.141033 | 37 |
| `zero_gyro` | Remove gyro-tagged cross blocks | 6.768476 | 117.981238 | 37 |
| `zero_plane` | Remove plane-to-other-state cross covariance | 6.742032 | 117.395549 | 37 |

`zero_gyro` does **not** disable IMU propagation or set angular velocity to zero. `zero_plane` retains marginal uncertainties. See `CrossAblation` and `apply_cross_ablation` in [research_joint_flow.h](../src/xq_fast_livo2/include/research_joint_flow.h), called by [research_joint_integration.cpp](../src/xq_fast_livo2/src/research_joint_integration.cpp). `joint_no_flow` is the primary control for the pixel update. The standalone serial prototype is not a sixth native replay arm.

## Metric and inputs

Position ATE uses **first-pair SE(3) alignment without fitting scale** and nearest truth by acquisition timestamp within 15 ms. The reference has 1,253 matched states, zero unmatched, maximum matching offset 9 ms. This is not best-fit whole-trajectory alignment. See [align_and_measure](../scripts/analyze_livo_replay.py) and the independent [joint evaluator](../scripts/analyze_joint_livo_replay.py).

The common sensor input is 24,144 messages: IMU 16,586; LiDAR 830; front/downward images 1,270 each; range 4,188. Publisher completion does not establish estimator completion: five counts, final acquisition stamps, image accounting and flush barriers must agree. The 37 accepted flow updates are actual state/covariance changes, not received range counts. Most range messages are not fused in this low-duty-cycle profile. Truth and noise-free range reference are evaluator-only.

## Required files

| Run | Evidence to inspect |
|---|---|
| CPU studies | `<output>/summary.json`: top-level PASS and six study entries PASS; inspect study logs/results |
| LIVO build | `<runtime>/server-build-result.json`: PASS, plus source/dependency/binary manifests |
| Replay | `<trial>/suite-result.json`: PASS; each arm's `run-result.json` and `processing-result.json` |
| Evaluation | `<evaluation>/report.json`: PASS; `arms.<method>.trajectory_metrics.ate_rms_m` is in metres; `actual_nonzero_state_and_covariance_updates` counts actual updates |
| Base Gazebo smoke | `<run>/result.json`: inspect `status`, `checks` and `scope`; this proxy smoke has no mission-status field |
| Advanced Gazebo scenario | `<run>/result.json`: inspect `acceptance_mode` and `mission_status` as well as top-level status |
| AP preparation | The emitted `run` contains `status.json`; PASS establishes the stated source/build preparation |
| AP smoke | Its own emitted `run` contains `status.json`; PASS is unarmed version/parameter evidence |

Display your new evaluation table:

```bash
EVALUATION=/absolute/path/to/your_evaluation
/usr/bin/python3 - "$EVALUATION/report.json" <<'PY'
import json, sys
report = json.load(open(sys.argv[1]))
print('status:', report['status'])
for name, arm in report['arms'].items():
    print(name, 'ATE RMS mm =', arm['trajectory_metrics']['ate_rms_m'] * 1000,
          'actual flow updates =', arm['actual_nonzero_state_and_covariance_updates'])
PY
```

Reference evidence:

- [Independent evaluation](../server/validation/livo/evaluation-report.json).
- [Suite and input accounting](../server/validation/livo/suite-result.json).
- [Fresh relocated LIVO validation](../server/LIVO_SERVER_VALIDATION.json).
- [Historical P15 chain SMOKE and retained failures](../server/advanced_simulation_validation.json).
- [Base build/simulation](../server/simulation_validation.json) and [AP 4.7.1 validation](../server/AP471_SERVER_VALIDATION.json).

These are reference evidence, not outputs from your new machine. Old absolute paths are provenance; generate fresh outputs instead of copying or rewriting a PASS report.

## Interpretation limits

The numbers describe one simulated flight under a local-plane and engineering noise model. They do not establish statistical superiority, globally calibrated covariance, integrity-bound coverage, arbitrary-terrain robustness, Orin timing or a new-estimator/AP complete mission. Preserve failures and report actual variation instead of selecting only the best run.
