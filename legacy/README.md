# Legacy: the C++ driver

[`cpp_driver/`](cpp_driver) is the C++ ROS 2 driver (version 0.1.0) as it was on
`main` at `f5d909c`, before the Python restructure. It is kept as a reference
only: it is not built (`COLCON_IGNORE`), not tested and not maintained. Its
original README is [`cpp_driver/Readme.md`](cpp_driver/Readme.md); the paths in
it are relative to the old repository root.

Known problems, fixed in `isss_core` (see [`docs/protocol.md`](../docs/protocol.md)):

- The bearing formula uses the 12.5 MHz sample clock as the carrier and is off by
  a factor of 512, so nearly every value clamps to ±90°.
- The published phase is `counts / 512 · π / 180`, a ninth of the true angle in
  radians.

For what changed for users of the old node, see
[Migrating from the C++ driver](../Readme.md#migrating-from-the-c-driver--010).
To build it anyway, check out the tag `cpp-driver-0.1.0`.
