//! This is a wrapper for the `quaternion-core` crate.
//! 
//! Provides quaternion operations and interconversion with several attitude representations.
//! Operator overloading allows implementation similar to mathematical expressions.
//! 
//! The supported operations are listed in the table below:
//! 
//! | ↓Left / Right→      | QuaternionWrapper               | Vector3Wrapper            | ScalarWrapper      |
//! |:---------------------:|:--------------------------------|:--------------------------|:-------------------|
//! | __QuaternionWrapper__ | `+`, `-`, `*`, `+=`, `-=`, `*=` | `+`, `-`, `*`             | `+`, `-`, `*`, `/` |
//! | __Vector3Wrapper__    | `+`, `-`, `*`                   | `+`, `-`, `*`, `+=`, `-=` | `+`, `-`, `*`, `/` |
//! | __ScalarWrapper__     | `+`, `-`, `*`                   | `+`, `-`, `*`             | `+`, `-`, `*`, `/`, `+=`, `-=`, `*=`, `/=` |
//! 
//! ## Versor
//! 
//! Versor refers to a Quaternion representing a rotation, the norm of which is 1.
//! 
//! The documentation for this crate basically writes Versor instead of Unit Quaternion, 
//! but the difference in usage is not clear.
//! Please think Versor = Unit Quaternion.

#![no_std]
#[cfg(feature = "std")]
extern crate std;

use num_traits::{Float, FloatConst};
use quaternion_core as quat;
pub use quat::{Vector3, Quaternion, DCM, RotationType, RotationSequence};

mod ops;

#[derive(Debug, Clone, Copy)]
pub struct QuaternionWrapper<T>(pub Quaternion<T>);

/// Treated as Pure Quaternion.
/// 
/// `QuaternionWrapper = ScalarWrapper + Vector3Wrapper`
#[derive(Debug, Clone, Copy)]
pub struct Vector3Wrapper<T>(pub Vector3<T>);

/// Treated as Real Quaternion.
/// 
/// `QuaternionWrapper = ScalarWrapper + Vector3Wrapper`
#[derive(Debug, Clone, Copy)]
pub struct ScalarWrapper<T>(pub T);

// ------------------------ Quaternion ------------------------ //
impl<T: Float + FloatConst> QuaternionWrapper<T> {
    /// Create a new Quaternion.
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper::new( (1.0, [0.0; 3]) );
    /// 
    /// // Or it could be written like this
    /// let q = QuaternionWrapper( (1.0, [0.0; 3]) );
    /// ```
    #[inline]
    pub fn new(q: Quaternion<T>) -> Self {
        Self(q)
    }

    /// Create a new Identity Quaternion
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q: QuaternionWrapper<f64> = QuaternionWrapper::new_identity();
    /// 
    /// let p: QuaternionWrapper<f64> = QuaternionWrapper::new((1.0, [0.0; 3]));
    /// 
    /// assert_eq!(q.unwrap(), p.unwrap());
    /// ```
    #[inline]
    pub fn new_identity() -> Self {
        Self( (T::one(), [T::zero(); 3]) )
    }

    /// Returns the `Quaternion<T>`.
    #[inline]
    pub fn unwrap(self) -> Quaternion<T> {
        self.0
    }

    /// Returns the scalar part of a quaternion.
    #[inline]
    pub fn get_scalar_part(self) -> ScalarWrapper<T> {
        ScalarWrapper( (self.0).0 )
    }

    /// Returns the vector part of a quaternion.
    #[inline]
    pub fn get_vector_part(self) -> Vector3Wrapper<T> {
        Vector3Wrapper( (self.0).1 )
    }

    /// Generate Versor by specifying rotation `angle`\[rad\] and `axis` vector.
    /// 
    /// The `axis` vector does not have to be a unit vector.
    /// 
    /// If you enter a zero vector, it returns an identity quaternion.
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Generates a quaternion representing the
    /// // rotation of π/2[rad] around the y-axis.
    /// let q = QuaternionWrapper::from_axis_angle([0.0, 1.0, 0.0], PI/2.0);
    /// 
    /// // Rotate the point.
    /// let r = q.point_rotation( Vector3Wrapper([2.0, 2.0, 0.0]) );
    /// 
    /// // Check if the calculation is correct.
    /// let diff = Vector3Wrapper([0.0, 2.0, -2.0]) - r;
    /// for val in diff.unwrap() {
    ///     assert!( val.abs() < 1e-12 );
    /// }
    /// ```
    #[inline]
    pub fn from_axis_angle(axis: Vector3<T>, angle: T) -> Self {
        Self( quat::from_axis_angle(axis, angle) )
    }

    /// Convert a DCM to a Versor representing 
    /// the `q v q*` rotation (Point Rotation - Frame Fixed).
    /// 
    /// When convert to a DCM representing `q* v q` rotation
    /// (Frame Rotation - Point Fixed) to a Versor, do the following:
    /// 
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// # let dcm = QuaternionWrapper((1.0, [0.0; 3])).to_dcm();
    /// let q = QuaternionWrapper::from_dcm(dcm).conj();
    /// ```
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Make these as you like.
    /// let v = Vector3Wrapper([1.0, 0.5, -8.0]);
    /// let q = QuaternionWrapper::from_axis_angle([0.2, 1.0, -2.0], PI/4.0);
    /// 
    /// // --- Point rotation --- //
    /// {
    ///     let m = q.to_dcm();
    ///     let q_check = QuaternionWrapper::from_dcm(m);
    ///     
    ///     let diff = (q - q_check).unwrap();
    ///     assert!( diff.0.abs() < 1e-12 );
    ///     assert!( diff.1[0].abs() < 1e-12 );
    ///     assert!( diff.1[1].abs() < 1e-12 );
    ///     assert!( diff.1[2].abs() < 1e-12 );
    /// }
    /// 
    /// // --- Frame rotation --- //
    /// {
    ///     let m = q.conj().to_dcm();
    ///     let q_check = QuaternionWrapper::from_dcm(m).conj();
    ///     
    ///     let diff = (q - q_check).unwrap();
    ///     assert!( diff.0.abs() < 1e-12 );
    ///     assert!( diff.1[0].abs() < 1e-12 );
    ///     assert!( diff.1[1].abs() < 1e-12 );
    ///     assert!( diff.1[2].abs() < 1e-12 );
    /// }
    /// ```
    #[inline]
    pub fn from_dcm(m: DCM<T>) -> Self {
        Self( quat::from_dcm(m) )
    }

    /// Convert Euler angles to Versor.
    /// 
    /// The type of rotation (Intrinsic or Extrinsic) is specified by `RotationType` enum, 
    /// and the rotation sequence (XZX, XYZ, ...) is specified by `RotationSequence` enum.
    /// 
    /// Each element of `angles` should be specified in the range: `[-2π, 2π]`.
    /// 
    /// Sequences: `angles[0]` ---> `angles[1]` ---> `angles[2]`
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// use quaternion_core::{RotationType::*, RotationSequence::XYZ};
    /// 
    /// let angles = [PI/6.0, 1.6*PI, -PI/4.0];
    /// let v = Vector3Wrapper([1.0, 0.5, -0.4]);
    /// 
    /// // Quaternions representing rotation around each axis.
    /// let x = QuaternionWrapper::from_axis_angle([1.0, 0.0, 0.0], angles[0]);
    /// let y = QuaternionWrapper::from_axis_angle([0.0, 1.0, 0.0], angles[1]);
    /// let z = QuaternionWrapper::from_axis_angle([0.0, 0.0, 1.0], angles[2]);
    /// 
    /// // ---- Intrinsic (X-Y-Z) ---- //
    /// // These represent the same rotation.
    /// let q_in = x * y * z;
    /// let e2q_in = QuaternionWrapper::from_euler_angles(Intrinsic, XYZ, angles);
    /// // Confirmation
    /// let a_in = q_in.point_rotation(v);
    /// let b_in = e2q_in.point_rotation(v);
    /// let diff_in = (a_in - b_in).unwrap();
    /// assert!( diff_in[0].abs() < 1e-12 );
    /// assert!( diff_in[1].abs() < 1e-12 );
    /// assert!( diff_in[2].abs() < 1e-12 );
    /// 
    /// // ---- Extrinsic (X-Y-Z) ---- //
    /// // These represent the same rotation.
    /// let q_ex = z * y * x;
    /// let e2q_ex = QuaternionWrapper::from_euler_angles(Extrinsic, XYZ, angles);
    /// // Confirmation
    /// let a_ex = q_ex.point_rotation(v);
    /// let b_ex = e2q_ex.point_rotation(v);
    /// let diff_ex = (a_ex - b_ex).unwrap();
    /// assert!( diff_ex[0].abs() < 1e-12 );
    /// assert!( diff_ex[1].abs() < 1e-12 );
    /// assert!( diff_ex[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn from_euler_angles(rt: RotationType, rs: RotationSequence, angles: Vector3<T>) -> Self {
        Self( quat::from_euler_angles(rt, rs, angles) )
    }

    /// Calculate the rotation `axis` (unit vector) and the rotation `angle`\[rad\] 
    /// around the `axis` from the Versor.
    /// 
    /// If identity quaternion is entered, `angle` returns zero and 
    /// the `axis` returns a zero vector.
    /// 
    /// Range of `angle`: `(-π, π]`
    #[inline]
    pub fn to_axis_angle(self) -> (Vector3Wrapper<T>, ScalarWrapper<T>) {
        let f = quat::to_axis_angle(self.0);
        ( Vector3Wrapper(f.0), ScalarWrapper(f.1) )
    }

    /// Convert a Versor to a DCM representing 
    /// the `q v q*` rotation (Point Rotation - Frame Fixed).
    /// 
    /// When convert to a DCM representing the 
    /// `q* v q` rotation (Frame Rotation - Point Fixed), do the following:
    /// 
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// # let q = QuaternionWrapper::<f64>::new_identity();
    /// let dcm = q.conj().to_dcm();
    /// ```
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Make these as you like.
    /// let v = Vector3Wrapper([1.0, 0.5, -8.0]);
    /// let q = QuaternionWrapper::from_axis_angle([0.2, 1.0, -2.0], PI/4.0);
    /// 
    /// // --- Point rotation --- //
    /// {
    ///     let m = q.to_dcm();
    /// 
    ///     let rm = v.matrix_product(m);
    ///     let rq = q.point_rotation(v);
    ///     let diff = (rm - rq).unwrap();
    ///     assert!( diff[0].abs() < 1e-12 );
    ///     assert!( diff[1].abs() < 1e-12 );
    ///     assert!( diff[2].abs() < 1e-12 );
    /// }
    /// 
    /// // --- Frame rotation --- //
    /// {
    ///     let m = q.conj().to_dcm();
    /// 
    ///     let rm = v.matrix_product(m);
    ///     let rq = q.frame_rotation(v);
    ///     let diff = (rm - rq).unwrap();
    ///     assert!( diff[0].abs() < 1e-12 );
    ///     assert!( diff[1].abs() < 1e-12 );
    ///     assert!( diff[2].abs() < 1e-12 );
    /// }
    /// ```
    #[inline]
    pub fn to_dcm(self) -> DCM<T> {
        quat::to_dcm(self.0)
    }

    /// Convert Versor to Euler angles.
    /// 
    /// The type of rotation (Intrinsic or Extrinsic) is specified by `RotationType` enum, 
    /// and the rotation sequence (XZX, XYZ, ...) is specified by `RotationSequence` enum.
    /// 
    /// ```
    /// # use quaternion_wrapper::{RotationType::Intrinsic, RotationSequence::XYZ, QuaternionWrapper};
    /// # let q = QuaternionWrapper::<f64>::new_identity();
    /// let angles = q.to_euler_angles(Intrinsic, XYZ);
    /// ```
    /// 
    /// Sequences: `angles[0]` ---> `angles[1]` ---> `angles[2]`
    /// 
    /// # Singularity
    /// 
    /// ## RotationType::Intrinsic
    /// 
    /// For Proper Euler angles (ZXZ, XYX, YZY, ZYZ, XZX, YXY), the singularity is reached 
    /// when the sine of the second rotation angle is 0 (angle = 0, ±π, ...), and for 
    /// Tait-Bryan angles (XYZ, YZX, ZXY, XZY, ZYX, YXZ), the singularity is reached when 
    /// the cosine of the second rotation angle is 0 (angle = ±π/2).
    /// 
    /// At the singularity, the third rotation angle is set to 0\[rad\].
    /// 
    /// ## RotationType::Extrinsic
    /// 
    /// As in the case of Intrinsic rotation, for Proper Euler angles, the singularity occurs 
    /// when the sine of the second rotation angle is 0 (angle = 0, ±π, ...), and for 
    /// Tait-Bryan angles, the singularity occurs when the cosine of the second rotation angle 
    /// is 0 (angle = ±π/2).
    /// 
    /// At the singularity, the first rotation angle is set to 0\[rad\].
    /// 
    /// # Examples
    /// 
    /// Depending on the rotation angle of each axis, it may not be possible to recover the 
    /// same rotation angle as the original. However, they represent the same rotation in 3D space.
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// use quaternion_wrapper::{RotationType::*, RotationSequence::XYZ};
    /// 
    /// let angles = Vector3Wrapper([PI/6.0, PI/4.0, PI/3.0]);
    /// 
    /// // ---- Intrinsic (X-Y-Z) ---- //
    /// let q_in = QuaternionWrapper::from_euler_angles(Intrinsic, XYZ, angles.unwrap());
    /// let e_in = q_in.to_euler_angles(Intrinsic, XYZ);
    /// let diff = (angles - e_in).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    /// 
    /// // ---- Extrinsic (X-Y-Z) ---- //
    /// let q_ex = QuaternionWrapper::from_euler_angles(Extrinsic, XYZ, angles.unwrap());
    /// let e_ex = q_ex.to_euler_angles(Extrinsic, XYZ);
    /// let diff = (angles - e_ex).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn to_euler_angles(self, rt: RotationType, rs: RotationSequence) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::to_euler_angles(rt, rs, self.0) )
    }

    /// Calculate a Versor to rotate from vector `a` to `b`.
    /// 
    /// If you enter a zero vector, it returns an identity quaternion.
    /// 
    /// # Example
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// let a = Vector3Wrapper::<f64>::new([1.5, -0.5, 0.2]);
    /// let b = Vector3Wrapper::<f64>::new([0.1, 0.6, 1.0]);
    /// 
    /// let q = QuaternionWrapper::rotate_a_to_b(a, b);
    /// let b_check = q.point_rotation(a);
    /// 
    /// let cross = b.cross(b_check).unwrap();
    /// assert!( cross[0].abs() < 1e-12 );
    /// assert!( cross[1].abs() < 1e-12 );
    /// assert!( cross[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn rotate_a_to_b(a: Vector3Wrapper<T>, b: Vector3Wrapper<T>) -> Self {
        Self( quat::rotate_a_to_b(a.0, b.0) )
    }

    /// Calculate the sum of each element of Quaternion or Vector3.
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper::<f64>::new( (1.0, [2.0, 3.0, 4.0]) );
    /// 
    /// assert!( (10.0 - q.sum().unwrap()).abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn sum(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::sum(self.0) )
    }

    /// Calculate `s*self + b`
    /// 
    /// If the `fma` feature is enabled, the FMA calculation is performed using the `mul_add` method. 
    /// If not enabled, it's computed by unfused multiply-add (s*a + b).
    #[inline]
    pub fn scale_add(self, s: ScalarWrapper<T>, b: QuaternionWrapper<T>) -> Self {
        Self( quat::scale_add(s.0, self.0, b.0) )
    }

    /// Hadamard product of Quaternion.
    /// 
    /// Calculate `self ∘ other`
    #[inline]
    pub fn hadamard(self, other: QuaternionWrapper<T>) -> Self {
        Self( quat::hadamard(self.0, other.0) )
    }

    /// Hadamard product and Addiction of Quaternion.
    /// 
    /// Calculate `a ∘ b + c`
    /// 
    /// If the `fma` feature is enabled, the FMA calculation is performed using the `mul_add` method. 
    /// If not enabled, it's computed by unfused multiply-add (s*a + b).
    #[inline]
    pub fn hadamard_add(self, b: QuaternionWrapper<T>, c: QuaternionWrapper<T>) -> Self {
        Self( quat::hadamard_add(self.0, b.0, c.0) )
    }

    /// Dot product of the quaternion.
    #[inline]
    pub fn dot(self, other: QuaternionWrapper<T>) -> ScalarWrapper<T> {
        ScalarWrapper( quat::dot(self.0, other.0) )
    }

    /// Calcurate the L2 norm of the quaternion.
    #[inline]
    pub fn norm(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::norm(self.0) )
    }

    /// Normalization of quaternion.
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// // This norm is not 1.
    /// let q = QuaternionWrapper::<f64>::new( (1.0, [2.0, 3.0, 4.0]) );
    /// assert!( (1.0 - q.norm().unwrap()).abs() > 1e-12 );
    /// 
    /// // Now that normalized, this norm is 1!
    /// let q_n = q.normalize();
    /// assert!( (1.0 - q_n.norm().unwrap()).abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn normalize(self) -> Self {
        Self( quat::normalize(self.0) )
    }

    /// Returns the conjugate of quaternion.
    #[inline]
    pub fn conj(self) -> Self {
        Self( quat::conj(self.0) )
    }

    /// Calculate the inverse of quaternion.
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper::<f64>::new( (1.0, [2.0, 3.0, 4.0]) );
    /// 
    /// // Identity quaternion
    /// let id = (q * q.inv()).unwrap();  // = (q.inv() * q).unwrap()
    /// 
    /// assert!( (id.0 - 1.0).abs() < 1e-12 );
    /// assert!( id.1[0].abs() < 1e-12 );
    /// assert!( id.1[1].abs() < 1e-12 );
    /// assert!( id.1[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn inv(self) -> Self {
        Self( quat::inv(self.0) )
    }

    /// Exponential function of quaternion
    #[inline]
    pub fn exp(self) -> Self {
        Self( quat::exp(self.0) )
    }

    /// Natural logarithm of quaternion.
    #[inline]
    pub fn ln(self) -> Self {
        Self( quat::ln(self.0) )
    }

    /// Natural logarithm of versor.
    /// 
    /// If it is guaranteed to be a versor, it is less computationally 
    /// expensive than the `.ln()` method. 
    /// 
    /// Only the vector part is returned since the real part is always zero.
    #[inline]
    pub fn ln_versor(self) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::ln_versor(self.0) )
    }

    /// Power function of quaternion.
    #[inline]
    pub fn pow(self, t: T) -> Self {
        Self( quat::pow(self.0, t) )
    }

    /// Power function of versor.
    /// 
    /// If it is guaranteed to be a versor, it is less computationally 
    /// expensive than the `.pow()` method. 
    #[inline]
    pub fn pow_versor(self, t: T) -> Self {
        Self( quat::pow_versor(self.0, t) )
    }

    /// Rotation of vector (Point Rotation - Frame Fixed)
    /// Rotation of point (Point Rotation - Frame Fixed)
    /// 
    /// `q v q*  (||q|| = 1)`
    /// 
    /// Since it is implemented with an optimized formula, 
    /// it can be calculated with the amount of operations shown in the table below:
    /// 
    /// | Operation    | Num |
    /// |:------------:|:---:|
    /// | Multiply     | 18  |
    /// | Add/Subtract | 12  |
    /// 
    /// # Example
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Make these as you like.
    /// let v = Vector3Wrapper([1.0, 0.5, -8.0]);
    /// let q = QuaternionWrapper::from_axis_angle([0.2, 1.0, -2.0], PI);
    /// 
    /// let r = q.point_rotation(v);
    /// 
    /// // This makes a lot of wasted calculations.
    /// let r_check = (q * v * q.conj()).get_vector_part();
    /// 
    /// let diff = (r - r_check).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn point_rotation(self, v: Vector3Wrapper<T>) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::point_rotation(self.0, v.0) )
    }

    /// Rotation of frame (Frame Rotation - Point Fixed)
    /// 
    /// `q* v q  (||q|| = 1)`
    /// 
    /// Since it is implemented with an optimized formula, 
    /// it can be calculated with the amount of operations shown in the table below:
    /// 
    /// | Operation    | Num |
    /// |:------------:|:---:|
    /// | Multiply     | 18  |
    /// | Add/Subtract | 12  |
    /// 
    /// # Example
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Make these as you like.
    /// let v = Vector3Wrapper([1.0, 0.5, -8.0]);
    /// let q = QuaternionWrapper::from_axis_angle([0.2, 1.0, -2.0], PI);
    /// 
    /// let r = q.frame_rotation(v);
    /// 
    /// // This makes a lot of wasted calculations.
    /// let r_check = (q.conj() * v * q).get_vector_part();
    /// 
    /// let diff = (r - r_check).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn frame_rotation(self, v: Vector3Wrapper<T>) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::frame_rotation(self.0, v.0) )
    }

    /// Lerp (Linear interpolation)
    /// 
    /// Generate a quaternion that interpolate the shortest path from `self` to `other` 
    /// (The norm of `self` and `other` must be 1).
    /// The argument `t (0 <= t <= 1)` is the interpolation parameter.
    /// 
    /// Normalization is not performed internally because 
    /// it increases the computational complexity.
    #[inline]
    pub fn lerp(self, other: QuaternionWrapper<T>, t: T) -> Self {
        Self( quat::lerp(self.0, other.0, t) )
    }

    /// Slerp (Spherical linear interpolation)
    /// 
    /// Generate a quaternion that interpolate the shortest path from `self` to `other`.
    /// The argument `t(0 <= t <= 1)` is the interpolation parameter.
    /// 
    /// The norm of `self` and `other` must be 1 (Versor).
    #[inline]
    pub fn slerp(self, other: QuaternionWrapper<T>, t: T) -> Self {
        Self( quat::slerp(self.0, other.0, t) )
    }
}

// ------------------------- Vector3 ------------------------- //
impl<T: Float> Vector3Wrapper<T> {
    /// Create a new Vector3Wrapper
    #[inline]
    pub fn new(v: Vector3<T>) -> Self {
        Self(v)
    }

    /// Returns the `Vector3<T>`
    #[inline]
    pub fn unwrap(self) -> Vector3<T> {
        self.0
    }

    /// Product of Vector3Wrapper and DCM.
    /// 
    /// `m * self`
    #[inline]
    pub fn matrix_product(self, m: DCM<T>) -> Self {
        Self( quat::matrix_product(m, self.0) )
    }

    /// Sum of the element of the vector.
    #[inline]
    pub fn sum(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::sum(self.0) )
    }

    /// Calculate `s*self + b`
    /// 
    /// If the `fma` feature is enabled, the FMA calculation is performed using the `mul_add` method. 
    /// If not enabled, it's computed by unfused multiply-add (s*a + b).
    #[inline]
    pub fn scale_add(self, s: ScalarWrapper<T>, b: Vector3Wrapper<T>) -> Self {
        Self( quat::scale_add(s.0, self.0, b.0) )
    }

    /// Hadamard product of vector.
    /// 
    /// Calculate `a ∘ b`
    #[inline]
    pub fn hadamard(self, other: Vector3Wrapper<T>) -> Self {
        Self( quat::hadamard(self.0, other.0) )
    }

    /// Hadamard product and Addiction of Vector.
    /// 
    /// Calculate `a ∘ b + c`
    /// 
    /// If the `fma` feature is enabled, the FMA calculation is performed using the `mul_add` method. 
    /// If not enabled, it's computed by unfused multiply-add (s*a + b).
    #[inline]
    pub fn hadamard_add(self, b: Vector3Wrapper<T>, c: Vector3Wrapper<T>) -> Self {
        Self( quat::hadamard_add(self.0, b.0, c.0) )
    }

    /// Dot product of the vector.
    #[inline]
    pub fn dot(self, other: Vector3Wrapper<T>) -> ScalarWrapper<T> {
        ScalarWrapper( quat::dot(self.0, other.0) )
    }

    /// Cross product of the vector.
    /// 
    /// `self × other`
    #[inline]
    pub fn cross(self, other: Vector3Wrapper<T>) -> Self {
        Self( quat::cross(self.0, other.0) )
    }

    /// Calcurate the L2 norm of the vector.
    #[inline]
    pub fn norm(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::norm(self.0) )
    }

    /// Normalization of vector.
    /// 
    /// If you enter a zero vector, it returns a zero vector.
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// // This norm is not 1.
    /// let v = Vector3Wrapper::<f64>::new([1.0, 2.0, 3.0]);
    /// assert!( (1.0 - v.norm().unwrap()).abs() > 1e-12 );
    /// 
    /// // Now that normalized, this norm is 1!
    /// let v_n = v.normalize();
    /// assert!( (1.0 - v_n.norm().unwrap()).abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn normalize(self) -> Self {
        Self( quat::normalize(self.0) )
    }

    /// Exponential function of vector.
    #[inline]
    pub fn exp(self) -> QuaternionWrapper<T> {
        QuaternionWrapper( quat::exp(self.0) )
    }
}

// --------------------- Scalar -------------------- //
impl<T: Float> ScalarWrapper<T> {
    /// Create a new ScalarWrapper
    #[inline]
    pub fn new(s: T) -> Self {
        Self(s)
    }

    /// Returns the `T`
    #[inline]
    pub fn unwrap(self) -> T {
        self.0
    }
}
