# Citation and attribution

Use an exact source revision and resource release when reporting a reproduction. The fixed public server reproduction version is:

- Software: **IMPACT research software**.
- Release: [`server-sim-2026.10.22`](https://github.com/Accelerate11/IMPACT/releases/tag/server-sim-2026.10.22).
- Source commit: [`5750e6e204b697349e7b08bc1aca5e0bee1d51a9`](https://github.com/Accelerate11/IMPACT/tree/5750e6e204b697349e7b08bc1aca5e0bee1d51a9).
- Repository: [Accelerate11/IMPACT](https://github.com/Accelerate11/IMPACT).
- Resource identities: [server/resources.json](server/resources.json).

This version identifies the archived reproduction release. If you use a later `main` checkout, report its actual `git rev-parse HEAD` and retain that run's input/configuration/build identities as well; do not label changed code as the fixed release.

An author-free BibTeX entry is provided because the paper author list has not been supplied. It cites the software artifact, not a published or accepted conference paper:

```bibtex
@misc{impact2026software,
  title        = {{IMPACT} research software},
  year         = {2026},
  howpublished = {GitHub software repository and reproduction release},
  url          = {https://github.com/Accelerate11/IMPACT/tree/5750e6e204b697349e7b08bc1aca5e0bee1d51a9},
  note         = {Version server-sim-2026.10.22; source commit 5750e6e204b697349e7b08bc1aca5e0bee1d51a9}
}
```

The English and Chinese files in [papers/](papers/README.md) are research drafts. Their presence does not establish conference submission, acceptance, publication or a DOI. Do not infer a paper author name from a GitHub username. Update bibliographic details only when the authors provide the final information.

When using the underlying estimators and planner, also cite their original methods as appropriate. Their project pages and the draft's editable `references.bib` provide the original references:

- [FAST-LIO / FAST-LIO2](https://github.com/hku-mars/FAST_LIO).
- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2); the ROS 2 integration used here is a separately pinned community port.
- [EGO-Planner / EGO-Planner-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm).
- [ArduPilot](https://github.com/ArduPilot/ardupilot) for the firmware platform, if relevant to the experiment.

The source paths, pins and existing license notices are indexed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Citation is attribution; it does not replace those notices or grant additional redistribution rights.
