# Phase evidence archive

This directory contains the lightweight, Git-safe evidence for the formally accepted P0-P8 runs.
Use `docs/PHASE_ARCHIVE_INDEX.md` as the human-readable entry point.

Each phase directory contains the relevant result summaries, metrics, graph audits, rosbag metadata,
configuration hashes, and isolation checks. `LARGE_ARTIFACTS.sha256` records the SHA-256, byte size,
and original WSL-relative path of rosbag / Gazebo recording files that are intentionally not committed.

Regenerate the archive from the frozen WSL results with:

```powershell
powershell.exe -ExecutionPolicy Bypass -File scripts/archive_phase_evidence.ps1
```

The archive script reads the formal result directories and never modifies them.
