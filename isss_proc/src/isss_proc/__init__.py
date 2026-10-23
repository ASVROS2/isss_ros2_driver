"""Processing of DeepVision ISSS pings: filters that turn phase and intensity into trusted soundings."""

from isss_proc.filters import (
    BiasStates,
    CrossTalk,
    FirstSurface,
    IntensityExcess,
    MostProbableAngle,
    RangeGate,
    clean_intensity,
    default_stages,
    estimate_leak,
)
from isss_proc.pipeline import Pipeline, Result, Stream, make_pipeline, viewer_keep
from isss_proc.swath import Swath

__all__ = [
    "BiasStates",
    "CrossTalk",
    "FirstSurface",
    "IntensityExcess",
    "MostProbableAngle",
    "Pipeline",
    "RangeGate",
    "Result",
    "Stream",
    "Swath",
    "clean_intensity",
    "default_stages",
    "estimate_leak",
    "make_pipeline",
    "viewer_keep",
]

__version__ = "0.1.0"
