# quaternion-wrapper

[![Latest version](https://img.shields.io/crates/v/quaternion-wrapper?color=orange&style=flat-square)](https://crates.io/crates/quaternion-wrapper)
[![Documentation](https://img.shields.io/docsrs/quaternion-wrapper/latest?color=brightgreen&style=flat-square&logo=docs.rs)](https://docs.rs/quaternion-wrapper)
![Minimum rustc](https://img.shields.io/badge/rustc-1.53+-red.svg?style=flat-square&logo=rust)
![License](https://img.shields.io/crates/l/quaternion-wrapper?color=blue&style=flat-square)

This is a wrapper for the 
[quaternion-core](https://crates.io/crates/quaternion-core) 
crate.

Provides quaternion operations and interconversion with several attitude representations.

Operator overloading allows implementation similar to mathematical expressions.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
quaternion-wrapper = "0.3"
```

For use in a `no_std` environment:

```toml
[dependencies.quaternion-wrapper]
version = "0.3"
default-features = false
features = ["libm"]
```

## Operator Overloading

Operator overloading allows operations between `QuaternionWrapper`, `Vector3Wrapper`, and `ScalarWrapper`.

The supported operations are listed in the table below:

| Left↓ / Right→      | QuaternionWrapper               | Vector3Wrapper            | ScalarWrapper      |
|:---------------------:|:--------------------------------|:--------------------------|:-------------------|
| __QuaternionWrapper__ | `+`, `-`, `*`, `+=`, `-=`, `*=` | `+`, `-`, `*`             | `+`, `-`, `*`, `/` |
| __Vector3Wrapper__    | `+`, `-`, `*`                   | `+`, `-`, `*`, `+=`, `-=` | `+`, `-`, `*`, `/` |
| __ScalarWrapper__     | `+`, `-`, `*`                   | `+`, `-`, `*`             | `+`, `-`, `*`, `/`, `+=`, `-=`, `*=`, `/=` |

To prevent implementation errors by users, the operation with `T` (`f32` or `f64`) is 
intentionally not implemented.
That is, `ScalarWrapper<f64> * QuaternionWrapper<f64>` can be calculated, 
but `f64 * QuaternionWrapper<f64>` cannot.

## Features

### fma

When this feature is enabled, the 
[mul_add](https://docs.rs/num-traits/0.2.15/num_traits/float/trait.Float.html#tymethod.mul_add) 
method will be used internally as much as possible.
That is, `(s * a) + b` will be expanded as `s.mul_add(a, b)` at compile time.

This crate uses the `mul_add` method mainly to improve calculation speed, but if the CPU does 
not support the `FMA` (Fused Multiply-Add) instruction or if the `libm` feature is 
enabled, then the calculation is performed by the software implementation.
In this case, it may be rather slower than if the `fma` feature is not enabled.

### libm

If you set `default-features=false` (do not import `std`), you must enable this feature.

In this case, mathematical functions (e.g. `sin`, `cos`, `sqrt` ...) are provided by 
[libm](https://crates.io/crates/libm) crate.

### norm-sqrt

By default, the `a.norm()` method is implemented in such a way that overflow and 
underflow are less likely to occur than with `dot(a, a).sqrt()`. However, if extremely 
large values are not input and underflow is not that much of a concern, 
`dot(a, a).sqrt()` is sufficient (and `dot(a, a).sqrt()` is faster than the default implementation in most cases).

## Example

`src/main.rs`:

```rust
use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};

const PI: f64 = std::f64::consts::PI;
const EPSILON: f64 = 1e-12;

fn main() {
    // Generates a quaternion representing the
    // rotation of π/2[rad] around the y-axis.
    let q = QuaternionWrapper::from_axis_angle([0.0, 1.0, 0.0], PI/2.0);

    // Point
    let v = Vector3Wrapper([2.0, 2.0, 0.0]);

    let result = (q * v * q.conj()).get_vector_part();
    //let result = q.point_rotation(v);  // <--- It could be written like this

    // Check if the calculation is correct.
    let true_val = Vector3Wrapper([0.0, 2.0, -2.0]);
    let diff: [f64; 3] = (true_val - result).unwrap();
    for val in diff {
        assert!(val.abs() < EPSILON);
    }
}
```

## License

Licensed under either of
[Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0)
or
[MIT License](https://opensource.org/licenses/MIT)
at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted 
for inclusion in the work by you, as defined in the Apache-2.0 license, shall 
be dual licensed as above, without any additional terms or conditions.
