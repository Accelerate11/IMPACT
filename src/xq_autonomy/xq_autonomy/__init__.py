"""Xuanqiong-X1 algorithm-level simulation components.

Every module in this package is explicitly a SIL proxy.  It validates
interfaces, state transitions, metrics and algorithmic invariants; it does not
claim to be the production FAST-LIO2 or EGO-Planner implementation.
"""

__all__ = [
    "exploration",
    "geometry",
    "localization",
    "mapping",
    "metrics",
    "network",
    "planning",
    "sentinel",
    "types",
]

