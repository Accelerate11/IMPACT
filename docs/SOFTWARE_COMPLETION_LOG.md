# Hardware-independent completion log

The requested target is the acceptance matrix in `SOFTWARE_COMPLETION_ACCEPTANCE.md`.
ArduCopter **4.7.1**, official commit `dbe792162d06cab66c3475fd5556bf7a120f119e`, is the current firmware target. Historical 4.5.7 runs remain historical results. No hardware setup, flashing, physical calibration, or flight is part of this phase.

## 2026-10-21: build and first actual integration

- The original paper archive and B0–B4 evidence, including the stopped B4 archive, passed the software-completion preservation checkpoint. They remain immutable.
- The 28-dimensional nonlinear extension (native 19 + cloned pose 6 + local plane 3) passed 313 independent core checks, including finite-difference geometry, 45,000 range-augmentation Monte Carlo draws, and equivalence to a direct 28-dimensional Joseph update. These are core implementation checks, not a completed replay or superiority experiment.
- The new `joint_livo_install` was built separately. Its complete sources and 284 resolved runtime libraries are bound in the build manifest. Original `livo_install/livo_build` contents passed before/after preservation checks.
- The original B1 five-input recording has 127 exact simultaneous camera/range seed opportunities. Camera pairs have 66 ms separation; seed opportunities have 660 ms separation. The first replay profile consumes a simultaneous noisy range exactly once to augment the local plane, then updates using the next two-view pixel observation. Other range samples are received and audited without being fused. Pixel covariance is an explicit engineering setting, not a calibrated sensor claim.
- The separate evaluator confirms an actual simulated flight segment in the recording: approximately 2.066 m vertical span and 2.078 m maximum horizontal displacement from its initial position. Truth/reference data are not loaded by the replay publisher or estimator.

### Preserved failed integration attempt

`joint_livo_integration_20261021T075403Z_311` failed at a camera epoch with no new LiDAR snapshot. The input bag was unchanged. The 15 Hz cameras produce camera epochs between 10 Hz instantaneous LiDAR scans; the prior replay used decimated cameras and had assumed every published epoch contained LiDAR points.

The opt-in five-input profile now permits IMU propagation and VIO against the existing map at such epochs. It records zero fresh LiDAR points, the last nonempty LIO state epoch and map age, skips the LIO measurement, and does not publish the old cloud as a new scan. It still requires a real initialized LiDAR map before publishing the first VIO state.

### Newly discovered propagation defects and baseline interpretation

Review of the actual IMU implementation found that `last_prop_end_time` lacked initialization, and empty-cloud returns could retain relative-time deskew history. The next revision explicitly initializes the propagation origin, checks finite/non-regressing epochs, and clears cloud-local history on an empty-cloud return.

The stationary initialization model defines the initial state at the measurement epoch where initialization completes. No integration during the initialization interval is claimed. Every new comparison mode uses the same corrected bootstrap and snapshot handling. `joint_no_flow` additionally uses the same SO(3) resets and auxiliary-state handling as `full` and is the primary control for isolating the contribution of optical flow. The `native` mode is an additional no-flow baseline; its reset handling differs.

Historical binaries, source snapshots, outputs, and figures are retained. Their measured results are not silently reinterpreted as results of the corrected implementation. The newly identified initialization defect means historical accuracy observations alone cannot establish the correctness of the current propagation implementation; the corrected code requires fresh behavior tests and replay results.

### AP 4.7.1 preparation

Official SITL compilation and a disarmed parameter/version smoke test passed. The complete parameter table, official runtime version and commit prefix were verified. Two earlier failed smoke attempts are retained. Migration includes `GPS1_TYPE/GPS2_TYPE`, `SIM_GPS1_ENABLE/SIM_GPS2_ENABLE`, generic MAVLink `VISO_TYPE=1`, and `ARMING_SKIPCHK=0` in place of the old arming-check parameter.

This is firmware preparation only. Stable transport timestamps, exact MAVROS subscriber discovery, current FCU readiness, and the complete AP 4.7.1 task remain separately required. A disarmed heartbeat or receipt-health bit is not proof of EKF fusion or successful flight.

## 2026-10-21: native replay, controls, and CMICA working draft

- Corrected native IMU behavior passed 24 actual-library checks. The corrected isolated LIVO binary is `e34f63669ffc0cc3b7a745c653776e65bc156e759d62a8b9f0fbf8edea6cc97d` (SHA-256). The first failed replay and earlier binary remain retained.
- `joint_livo_integration_20261021T080222Z_530` and `joint_livo_controls_20261021T080835Z_318`, under the research workspace experiment results, completed independent audits. Each of five modes uses the same original B1 sensor records, source index and corrected binary; each produces 1,253 evaluated epochs. The full model applies 37 nonzero corrections to the actual native state and covariance. Of 123 completed pairs, 47 fail the predicted-translation gate and 39 have no surviving tracks. All are retained.
- RMS / maximum position errors in millimetres: native 7.368785 / 121.779016; joint/no-flow 7.203131 / 117.431185; full 6.644720 / 116.141033; zero gyro cross 6.768476 / 117.981238; zero plane cross 6.742032 / 117.395549. These are descriptive results from one simulated flight. The matched flow control is joint/no-flow; native reset handling differs. There is no multiseed superiority or calibrated-integrity claim.
- AP 4.7.1 isolated autonomy build passed 769 tests, version helpers 32 checks, and a ROS-only fixture 8 checks. Two real unarmed MAVROS/Gazebo trials matched 557 synthetic-source and 585 FAST-LIO-source outputs to ordered wire/VISP/VISV records. The synthetic two-second pause revoked readiness and required warm-up again. These lean trials do not demonstrate new-fusion closed-loop flight, EKF fusion, or repair of the old full-load outage. Gazebo shutdown escalation remains recorded.
- The strong serial prototype has 166 standalone checks but is not integrated in this replay cohort. Degradation selection tooling is not a completed multiseed/degradation experiment. The complete AP 4.7.1 mission and task-level revalidation remain pending.
- At the user's request, priority moved to a CMICA 2026 Winter working draft. `output/pdf/IMPACT_CMICA2026_Draft_20261021.pdf` and `output/pdf/impact_cmica2026_draft_20261021/` contain the English main paper and full mathematical appendices. Author details are deliberately pending. The paper distinguishes historical planning results, new nonlinear replay and unarmed AP evidence.
- Final legacy-results preservation checkpoint: `runs/paper_cmica_preservation_20261021T145202Z/report.json`, PASS. It checks the old paper baseline, B0–B4, interrupted B4 files and existing figures without modifying those archives.
