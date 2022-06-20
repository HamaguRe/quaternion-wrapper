# quaternion-wrapper

This is a wrapper for the 
[quaternion-core](https://crates.io/crates/quaternion-core) 
crate.

Provides quaternion operations and interconversion with several attitude representations.

Operator overloading allows implementation similar to mathematical expressions.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
quaternion-wrapper = "0.2"
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

`Cargo.toml`:

```toml
[dependencies.quaternion-wrapper]
version = "0.2"

# Uncomment if you wish to use FMA.
#features = ["fma"]

# Uncomment if you wish to use in "no_std" environment.
#default-features = false
#features = ["libm"]
```

### fma

This library uses the 
[mul_add](https://doc.rust-lang.org/std/primitive.f64.html#method.mul_add) 
method mainly to improve the performance, but by default it is replace with a unfused multiply-add 
(`s*a + b`) . If you wish to use mul_add method, enable the `fma` feature.

If your CPU does not support FMA instructions, or if you use `libm` (running in no_std 
environment), enabling the `fma` feature may cause slowdown of computation speed. Also, 
due to rounding error, results of `s.mul_add(a, b)` and `s*a + b` will not match perfectly.

### libm & default-features = false

These options allow for use in the `no_std` environment. 
In this case, mathematical functions (e.g. sin, cos, sqrt ...) are provided by 
[libm](https://crates.io/crates/libm).

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
