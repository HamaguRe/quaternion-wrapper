# Version 0.4.0 (2026-09-03)

* Updated `quaternion-core` to version 0.6.2 and `num-traits` to version 0.2.19.
* Added `convert_handedness`, `from_rotation_vector`, `to_rotation_vector`, `sqrt`, and `sqrt_versor`, and re-exported the `Axis` enum.
* Removed the interpolation parameter from `rotate_a_to_b_shortest` to match `quaternion-core`.
* Changed vector arguments in `from_axis_angle`, `from_euler_angles`, and `from_rotation_vector` from `Vector3<T>` to `Vector3Wrapper<T>` for API consistency.
* Relaxed trait bounds to match `quaternion-core`. Constructors and accessors no longer require `Float`, and `FloatConst` is required only where necessary.
* Revised the API documentation and examples to match `quaternion-core` and the wrapper types.
* Set the minimum supported Rust version to 1.60.
