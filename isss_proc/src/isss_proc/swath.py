"""The data every processing stage works on: a stack of pings and its calibration."""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property

import numpy as np

from isss.config import SonarConfig
from isss.diagnostics import PingArrays


@dataclass
class Swath:
    """Consecutive pings of one sample count and decimation, with the calibration to read them.

    ``arrays`` comes from any source ``isss`` reads (a live driver, a ``.bin``
    log, an exported ``.nc``, an old-driver bag), so a stage never sees a file
    format. Side 0 is port, 1 starboard, as in ``isss.diagnostics``.
    """

    arrays: PingArrays
    config: SonarConfig

    @property
    def n_pings(self) -> int:
        return self.arrays.n_pings

    @property
    def num_samples(self) -> int:
        return self.arrays.num_samples

    @property
    def intensity(self) -> np.ndarray:
        """(n, samples, 2) uint8 RSS."""
        return self.arrays.intensity

    @property
    def counts(self) -> np.ndarray:
        """(n, samples, 2) int16 signed phase counts."""
        return self.arrays.counts

    @cached_property
    def slant_range(self) -> np.ndarray:
        """(samples,) slant range of each bin [m]."""
        return self.arrays.slant_range(self.config)

    @cached_property
    def bin_size(self) -> float:
        """Slant range per bin [m]."""
        r = self.slant_range
        return float(r[1] - r[0]) if r.size > 1 else 1.0

    @cached_property
    def depression(self) -> np.ndarray:
        """(n, samples, 2) depression [rad] from the phase model; NaN where it has no solution."""
        return self.arrays.depression(self.config)

    def select(self, pings: slice) -> Swath:
        return Swath(self.arrays.select(pings), self.config)
