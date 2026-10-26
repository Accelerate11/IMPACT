# Reproduction troubleshooting / 复现排错

[中文入口](../README.md) · [English](../README.en.md) · [Expected results](REFERENCE_RESULTS.md)

Use system `/usr/bin/python3` on Ubuntu 22.04 (Python 3.10). Keep failed directories. Install/update dependencies before bound builds, and stop runs before changing source or system packages.

| Symptom / 现象 | Inspect / 查看 | Next action / 处理 |
|---|---|---|
| Want to understand the algorithm without ROS | `install_dependencies.sh --check --profile offline` | Run the six CPU studies. They need neither ROS/Gazebo nor released recording data. |
| Missing CMake package or Python module | Installer checks and the first failing compiler/import line | Use the appropriate dependency profile with system Python. Presence checks do not replace build/runtime validation. |
| System check passes but plain python3 imports fail | Compare `command -v python3` with `/usr/bin/python3` | Leave Conda/venv before ROS builds. The check uses system Python; a different Python selected by PATH is a different environment. |
| Output or LIVO runtime already exists | The path in the error | Use a new output/runtime; keep the previous attempt. Prepare/replay/evaluate must refer to the same new LIVO build. |
| Source/dependency identity mismatch | Commit, package versions and retained build log | Rebuild into a new prefix. Do not edit checksums or copy old manifests. |
| Old `/home/accelerate/...` in a JSON | Is it reference evidence or a new index? | Historical evidence keeps original paths. Use prepare to generate a new local index from the B1 bag. |
| Resource download fails | Completed parts and partial files in `server_runtime/downloads` | Rerun fetch. Verified complete parts are reused; a failed part is downloaded again. This is part-level reuse, not byte-level resume. urllib supports standard proxy environment variables. |
| Existing data/third_party without an installation manifest | Prior extraction/interruption | Keep it for diagnosis and use a new clone/root. Fetch refuses to overwrite an unverified tree. |
| Resource verification changes after Git commands | Vendored `.git/index` fingerprints | Inspect immutable vendor sources with `GIT_OPTIONAL_LOCKS=0 git -C third_party/ardupilot status`. Build tools work on copies. Altered resource trees require a fresh verified installation. |
| DDS discovery or input completion fails | Run-local transport config, node graph, counts and tails | LIVO needs a non-loopback IPv4 route. `ip -j -4 route get 1.1.1.1` checks it without sending a packet. Use isolated ROS domains; publisher exit is not estimator completion. |
| Headless Gazebo has no camera/LiDAR | EGL/renderer logs and sensor counts | GPU sensors still require a rendering backend. Try `--renderer software`; an empty scene is not sensor validation. |
| AP port busy or heartbeat absent | Explicit base port/instance, SITL and observer logs | Use free ports. The wrapper's base port is absolute; do not add the instance offset again. |
| Gazebo PASS but mission incomplete | `acceptance_mode`, `mission_status`, original evaluator | SMOKE verifies the stated running chain. Use `--require-mission-complete` for the stronger gate; keep failed/incomplete mission status. |
| Ctrl+C or timeout during CPU study | Summary and study log | The current runner records interruption/failure and stops its owned process group. Start a new output after diagnosis. The original reference tag predates this fix. |

## Transfer released resources without a server download

The resource release publishes **18 parts**, rather than a whole-resource asset. Download `.part-001` through `.part-018` and `SHA256SUMS.resources-v2` from the [release](https://github.com/Accelerate11/IMPACT/releases/tag/server-sim-2026.10.22), putting them in one directory. On Ubuntu/Bash:

```bash
sha256sum -c SHA256SUMS.resources-v2
cat IMPACT_Server_Resources_20261022_v2.tar.gz.part-{001..018} \
  > IMPACT_Server_Resources_20261022_v2.tar.gz
sha256sum IMPACT_Server_Resources_20261022_v2.tar.gz
# Expected whole-archive SHA-256:
# e12ed7706e7bb8cdb70b449d21c038789937ef48fac0d16f7752403f4c8e9869
# Now run the installer from your source checkout, with the absolute archive path.
cd /absolute/path/to/IMPACT
/usr/bin/python3 server/fetch_resources.py --root "$PWD" \
  --archive /absolute/path/to/IMPACT_Server_Resources_20261022_v2.tar.gz
```

Fetch checks the whole archive, authenticated inventory and every resource entry. This archive provides data/third-party source, not Ubuntu/ROS packages or compiled overlays. A GitHub source-code ZIP is not the resource archive.

For support use [GitHub Issues](https://github.com/Accelerate11/IMPACT/issues). Include commit, OS/architecture, Python/ROS/Gazebo versions, exact command, result/summary JSON and a bounded failing-log excerpt. State whether you ran a synthetic study, recorded replay, Gazebo smoke, mission gate or hardware; they are different validation levels.
