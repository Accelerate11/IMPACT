# Component licenses and source notices

This index records the license declarations and notices present in the repository and its fixed reproduction resources, checked on 2026-10-26. It does not assign a single license to the workspace, relicense third-party code, or grant a new license to the paper drafts and recorded data. Existing copyright headers and component license texts remain applicable.

## ROS packages in this repository

The table distinguishes a package's `package.xml` declaration from a separate license text. A declaration is not a substitute for the referenced text and file-level notices. The six EGO packages share their parent directory's license and provenance files.

| ROS package | Declared license | License text / source notice in this checkout |
|---|---|---|
| `impact_fault_injection` | Apache-2.0 | [package.xml](src/impact_fault_injection/package.xml); no separate component LICENSE file |
| `xq_autonomy` | Apache-2.0 | [package.xml](src/xq_autonomy/package.xml); no separate component LICENSE file |
| `xq_fast_lio` | GPL-2.0-only | [LICENSE](src/xq_fast_lio/LICENSE), [PROVENANCE.md](src/xq_fast_lio/PROVENANCE.md); retain BSD-3-Clause notices in individual mapping/IKFoM files |
| `xq_fast_livo2` | GPL-2.0 | [LICENSE](src/xq_fast_livo2/LICENSE), [PROVENANCE.md](src/xq_fast_livo2/PROVENANCE.md); keep the recorded upstream declaration and individual notices |
| `xq_gz_assets` | Apache-2.0 | [package.xml](src/xq_gz_assets/package.xml); no separate component LICENSE file; referenced external models have their own notices |
| `xq_gz_bridge` | Apache-2.0 | [package.xml](src/xq_gz_bridge/package.xml); no separate component LICENSE file |
| `xq_livox_interfaces` | Apache-2.0 | [package.xml](src/xq_livox_interfaces/package.xml); no separate component LICENSE file |
| `xq_sim_bringup` | Apache-2.0 | [package.xml](src/xq_sim_bringup/package.xml); no separate component LICENSE file |
| `xq_sim_interfaces` | Apache-2.0 | [package.xml](src/xq_sim_interfaces/package.xml); no separate component LICENSE file |
| `bspline_opt` | GPL-3.0-only | [shared EGO LICENSE](src/xq_ego_planner/LICENSE), [provenance](src/xq_ego_planner/PROVENANCE.md), [package.xml](src/xq_ego_planner/bspline_opt/package.xml) |
| `ego_planner` | GPL-3.0-only | [shared EGO LICENSE](src/xq_ego_planner/LICENSE), [provenance](src/xq_ego_planner/PROVENANCE.md), [package.xml](src/xq_ego_planner/ego_planner/package.xml) |
| `path_searching` | GPL-3.0-only | [shared EGO LICENSE](src/xq_ego_planner/LICENSE), [provenance](src/xq_ego_planner/PROVENANCE.md), [package.xml](src/xq_ego_planner/path_searching/package.xml) |
| `plan_env` | GPL-3.0-only | [shared EGO LICENSE](src/xq_ego_planner/LICENSE), [provenance](src/xq_ego_planner/PROVENANCE.md), [package.xml](src/xq_ego_planner/plan_env/package.xml) |
| `quadrotor_msgs` | GPL-3.0-only | [shared EGO LICENSE](src/xq_ego_planner/LICENSE), [provenance](src/xq_ego_planner/PROVENANCE.md), [package.xml](src/xq_ego_planner/quadrotor_msgs/package.xml) |
| `traj_utils` | GPL-3.0-only | [shared EGO LICENSE](src/xq_ego_planner/LICENSE), [provenance](src/xq_ego_planner/PROVENANCE.md), [package.xml](src/xq_ego_planner/traj_utils/package.xml) |

The FAST-LIO package metadata was corrected from its inherited BSD label to match the bundled GPL v2 text. This does not remove the BSD notices on constituent files. Its original local ROS 2 snapshot had no Git metadata, so its precise upstream revision remains **UNKNOWN**. The historical source path in its provenance identifies the original snapshot; it is not a server runtime dependency. Old validation reports and release tags retain their original metadata.

The following upstream identities are recorded for the source actually vendored here:

- FAST-LIO algorithm origin: [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO). The precise ROS 2 snapshot used by this package is described above; today's upstream `main` is not substituted for that unknown revision.
- FAST-LIVO2 algorithm origin: [hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2). The copied ROS 2 source is the community [Robotic-Developer-Road/FAST-LIVO2](https://github.com/Robotic-Developer-Road/FAST-LIVO2) port at `837b7bbc1431cb04cf936528e52c83c835efba8e`, with local changes documented in its provenance. It is not an author-maintained ROS 2 branch.
- EGO source: [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm), branch `ros2_version`, commit `23a8d5a191711dd65633df689b0b37ac07718416`. Only the six packages listed above are included; local integration patches are documented in the EGO provenance file.

## Fixed resources outside the ordinary Git clone

`data/` and `third_party/` are downloaded from [release `server-sim-2026.10.22`](https://github.com/Accelerate11/IMPACT/releases/tag/server-sim-2026.10.22). The archive, its parts and its complete inventory are authenticated by [server/resources.json](server/resources.json). Use [the downloader](server/fetch_resources.py) to install them. Paths below exist after installation; their upstream license files and submodule notices are retained in the resource archive.

| Resource | Fixed source identity | Existing license / notice location |
|---|---|---|
| Sophus | [strasdat/Sophus](https://github.com/strasdat/Sophus), `de0f8d3d92bf776271e16de56d1803940ebccab9` | MIT text at `third_party/livo/Sophus/LICENSE.txt` |
| RPG Vikit ROS 2 fork | [Robotic-Developer-Road/rpg_vikit](https://github.com/Robotic-Developer-Road/rpg_vikit), `4b7abc838f5d2ca9137f70f122eaaeff9eaf0f50` | `vikit_common` and `vikit_ros` manifests declare GPLv3; `vikit_py` has a placeholder license declaration; the fork's README additionally states an education/research-only, noncommercial policy. No standalone LICENSE file was found in this pinned export. These statements are recorded together, not resolved or replaced by this index. |
| ArduPilot | [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot), official `Copter-4.7.1`, `dbe792162d06cab66c3475fd5556bf7a120f119e` | GPL v3 text at `third_party/ardupilot/COPYING.txt`; individual files and all recursive submodules retain their own licenses |
| ArduPilot Gazebo plugin and models | [ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo); actual copied source revision **UNKNOWN_SOURCE_SNAPSHOT** | LGPL v3 text at `third_party/ardupilot_gazebo/LICENSE.md`; preserve individual plugin, model, mesh and texture notices, including Iris/PX4 attribution |
| B1 recording | Project-generated Gazebo simulation recording; six SQLite partitions plus metadata | Data provenance and immutable hashes in [coverage_manifest.json](server/coverage_manifest.json) and [LIVO runbook](server/LIVO_RUNBOOK.md). This index does not assign a new data license or relabel the recording as hardware data. |

The Vikit fork's mixed license statements should not be described as unrestricted commercial permission. Reproduction instructions target the stated research use and retain the pinned source and its notices. Dependency build details and patches are recorded in [livo_portability.json](server/livo_portability.json); ArduPilot/plugin export identities are in [ap471_portability.json](server/ap471_portability.json).

## Paper materials and system dependencies

- The editable paper ZIPs in [papers/](papers/README.md) retain the IEEEtran class's embedded copyright/license notice. The official template download and unchanged archive identities are documented in [the template source record](papers/templates/CMICA_IEEE_LaTeX_Template_20261022.source.json).
- The Chinese paper ZIP carries its plotting font and `fonts/DroidSansFallback-COPYRIGHT.txt`. Those font notices do not define the license of the paper text or the complete repository.
- ROS, Gazebo, Eigen, PCL, OpenCV, Python packages and other system dependencies are installed separately. Their distribution/package notices apply; installing them does not change the licenses of this source tree. See [the installer](server/install_dependencies.sh) for the actual dependency list.

For software version citation and attribution to the underlying methods, see [CITATION.md](CITATION.md).
