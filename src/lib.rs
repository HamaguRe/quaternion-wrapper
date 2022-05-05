//! This is a wrapper for the "quaternion" library.

#![no_std]
#[cfg(feature = "std")]
extern crate std;

use core::ops::{Add, AddAssign, Sub, SubAssign, Neg, Mul, MulAssign};
use num_traits::{Float, FloatConst};
use quaternion as quat;
use quat::FloatSimd;

pub use quat::{Vector3, Quaternion, DCM};

#[derive(Debug, Clone, Copy)]
pub struct QuaternionWrapper<T>(pub Quaternion<T>);

/// Treated as Pure Quaternion.
#[derive(Debug, Clone, Copy)]
pub struct Vector3Wrapper<T>(pub Vector3<T>);

/// Treated as Real Quaternion.
#[derive(Debug, Clone, Copy)]
pub struct ScalarWrapper<T>(pub T);

// ------------------------ Quaternion ------------------------ //
impl<T: Float + FloatConst + FloatSimd<T>> QuaternionWrapper<T> {
    /// Create a new QuaternionWrapper.
    #[inline]
    pub fn new(q: Quaternion<T>) -> Self {
        Self(q)
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

    /// Generate Versor by specifying rotation `angle`\[rad\] and axis vector.
    /// 
    /// The `axis` does not have to be a unit vector.
    /// 
    /// If you enter a zero vector, it returns an identity quaternion.
    #[inline]
    pub fn from_axis_angle(axis: Vector3<T>, angle: T) -> Self {
        Self( quat::from_axis_angle(axis, angle) )
    }

    /// Generate the versor (unit quaternion) from direction cosine matrix,
    /// representing rotation of position vector.
    #[inline]
    pub fn from_dcm(m: DCM<T>) -> Self {
        Self( quat::from_dcm(m) )
    }

    /// Convert Euler angles to quaternion.
    /// 
    /// Angle/Axis sequences is `[yaw, pitch, roll] / [z, y, x]`.
    #[inline]
    pub fn from_euler_angles(ypr: Vector3<T>) -> Self {
        Self( quat::from_euler_angles(ypr) )
    }

    /// Compute the rotation `axis` (unit vector) and the rotation `angle`\[rad\] 
    /// around the axis from the versor.
    #[inline]
    pub fn to_axis_angle(self) -> (Vector3<T>, T) {
        quat::to_axis_angle(self.0)
    }

    /// Convert from quaternions to direction cosines matrix.
    #[inline]
    pub fn to_dcm(self) -> DCM<T> {
        quat::to_dcm(self.0)
    }

    /// Convert from quaternions to euler angles.
    /// 
    /// Angle/Axis sequences is `[yaw, pitch, roll] / [z, y, x]`.
    #[inline]
    pub fn to_euler_angles(self) -> Vector3<T> {
        quat::to_euler_angles(self.0)
    }

    /// Sum of each element of the quaternion.
    #[inline]
    pub fn sum(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::sum(self.0) )
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

    /// Returns the normalized quaternion.
    #[inline]
    pub fn normalize(self) -> Self {
        Self( quat::normalize(self.0) )
    }

    /// Returns the conjugate of quaternion.
    #[inline]
    pub fn conj(self) -> Self {
        Self( quat::conj(self.0) )
    }

    /// Returns the inverse of quaternion.
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
    /// expensive than the `.ln()` method. 
    #[inline]
    pub fn pow_versor(self, t: T) -> Self {
        Self( quat::pow_versor(self.0, t) )
    }

    /// Rotation of vector (Point Rotation - Frame Fixed)
    /// 
    ///  `q v q*  (||q|| = 1)`
    #[inline]
    pub fn vector_rotation(self, v: Vector3Wrapper<T>) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::vector_rotation(self.0, v.0) )
    }

    /// Rotation of frame (Frame Rotation - Point Fixed)
    /// 
    /// `q* v q  (||q|| = 1)`
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

// H + H
impl<T: FloatSimd<T>> Add for QuaternionWrapper<T> {
    type Output = Self;
    fn add(self, other: QuaternionWrapper<T>) -> Self {
        Self( quat::add(self.0, other.0) )
    }
}

// H += H
impl<T: FloatSimd<T> + Copy> AddAssign for QuaternionWrapper<T> {
    fn add_assign(&mut self, other: QuaternionWrapper<T>) {
        *self = Self( quat::add(self.0, other.0) );
    }
}

// H + R^3
impl<T: Float> Add<Vector3Wrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn add(self, other: Vector3Wrapper<T>) -> Self {
        Self(( (self.0).0, quat::add_vec((self.0).1, other.0) ))
    }
}

// H + R
impl<T: Float> Add<ScalarWrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn add(self, other: ScalarWrapper<T>) -> Self {
        Self(( (self.0).0 + other.0, (self.0).1 ))
    }
}

// H - H
impl<T: FloatSimd<T>> Sub for QuaternionWrapper<T> {
    type Output = Self;
    fn sub(self, other: QuaternionWrapper<T>) -> Self {
        Self( quat::sub(self.0, other.0) )
    }
}

// H -= H
impl<T: FloatSimd<T> + Copy> SubAssign for QuaternionWrapper<T> {
    fn sub_assign(&mut self, other: QuaternionWrapper<T>) {
        *self = Self( quat::sub(self.0, other.0) );
    }
}

// -H
impl<T: FloatSimd<T>> Neg for QuaternionWrapper<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Self( quat::negate(self.0) )
    }
}

// H - R^3
impl<T: Float> Sub<Vector3Wrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn sub(self, other: Vector3Wrapper<T>) -> Self {
        Self(( (self.0).0, quat::sub_vec((self.0).1, other.0) ))
    }
}

// H - R
impl<T: Float> Sub<ScalarWrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn sub(self, other: ScalarWrapper<T>) -> Self {
        Self(( (self.0).0 - other.0, (self.0).1 ))
    }
}

// H * H
impl<T: Float + FloatSimd<T>> Mul for QuaternionWrapper<T> {
    type Output = Self;
    fn mul(self, other: QuaternionWrapper<T>) -> Self {
        Self( quat::mul(self.0, other.0) )
    }
}

// H *= H
impl<T: Float + FloatSimd<T>> MulAssign for QuaternionWrapper<T> {
    fn mul_assign(&mut self, other: QuaternionWrapper<T>) {
        *self = Self( quat::mul(self.0, other.0) );
    }
}

// H * R^3
impl<T: Float + FloatSimd<T>> Mul<Vector3Wrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn mul(self, other: Vector3Wrapper<T>) -> Self {
        Self((
            -quat::dot_vec((self.0).1, other.0), 
            quat::scale_add_vec((self.0).0, other.0, quat::cross_vec((self.0).1, other.0)) 
        ))
    }
}

// H * R
impl<T: FloatSimd<T>> Mul<T> for QuaternionWrapper<T> {
    type Output = Self;
    fn mul(self, coef: T) -> Self {
        Self( quat::scale(coef, self.0) )
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

    /// Sum of the element of the vector.
    #[inline]
    pub fn sum(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::sum_vec(self.0) )
    }

    /// Dot product of the vector.
    #[inline]
    pub fn dot(self, other: Vector3Wrapper<T>) -> ScalarWrapper<T> {
        ScalarWrapper( quat::dot_vec(self.0, other.0) )
    }

    /// Cross product of the vector.
    /// 
    /// `self × other`
    #[inline]
    pub fn cross(self, other: Vector3Wrapper<T>) -> Self {
        Self( quat::cross_vec(self.0, other.0) )
    }

    /// Calcurate the L2 norm of the vector.
    #[inline]
    pub fn norm(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::norm_vec(self.0) )
    }

    /// Returns the normalized vector.
    #[inline]
    pub fn normalize(self) -> Self {
        Self( quat::normalize_vec(self.0) )
    }

    /// Exponential function of vector.
    #[inline]
    pub fn exp(self) -> QuaternionWrapper<T> {
        QuaternionWrapper( quat::exp_vec(self.0) )
    }
}

// R^3 + R^3
impl<T: Float> Add for Vector3Wrapper<T> {
    type Output = Self;
    fn add(self, other: Vector3Wrapper<T>) -> Self {
        Self( quat::add_vec(self.0, other.0) )
    }
}

// R^3 += R^3
impl<T: Float> AddAssign for Vector3Wrapper<T> {
    fn add_assign(&mut self, other: Vector3Wrapper<T>) {
        *self = Self( quat::add_vec(self.0, other.0) )
    }
}

// R^3 + H
impl<T: Float> Add<QuaternionWrapper<T>> for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn add(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper(( (other.0).0, quat::add_vec(self.0, (other.0).1) ))
    }
}

// R^3 + R
impl<T: Float> Add<ScalarWrapper<T>> for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn add(self, other: ScalarWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper(( other.0, self.0 ))
    }
}

// R^3 - R^3
impl<T: Float> Sub for Vector3Wrapper<T> {
    type Output = Self;
    fn sub(self, other: Vector3Wrapper<T>) -> Self {
        Self( quat::sub_vec(self.0, other.0) )
    }
}

// R^3 -= R^3
impl<T: Float> SubAssign for Vector3Wrapper<T> {
    fn sub_assign(&mut self, other: Vector3Wrapper<T>) {
        *self = Self( quat::sub_vec(self.0, other.0) )
    }
}

// -R
impl<T: Float> Neg for Vector3Wrapper<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Self( quat::negate_vec(self.0) )
    }
}

// R^3 - H
impl<T: Float> Sub<QuaternionWrapper<T>> for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn sub(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper(( -(other.0).0, quat::sub_vec(self.0, (other.0).1) ))
    }
}

// R^3 - R
impl<T: Float> Sub<ScalarWrapper<T>> for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn sub(self, other: ScalarWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper(( -other.0, self.0 ))
    }
}

// R^3 * R^3
impl<T: Float> Mul for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn mul(self, other: Vector3Wrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper( quat::mul_vec(self.0, other.0) )
    }
}

// R^3 * H
impl<T: Float + FloatSimd<T>> Mul<QuaternionWrapper<T>> for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn mul(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper((
            -quat::dot_vec(self.0, (other.0).1),
            quat::scale_add_vec( (other.0).0, self.0, quat::cross_vec(self.0, (other.0).1) )
        ))
    }
}

// R^3 * R
impl<T: Float> Mul<ScalarWrapper<T>> for Vector3Wrapper<T> {
    type Output = Self;
    fn mul(self, other: ScalarWrapper<T>) -> Self {
        Self( quat::scale_vec(other.0, self.0) )
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

// R + R
impl<T: Float> Add for ScalarWrapper<T> {
    type Output = Self;
    fn add(self, other: ScalarWrapper<T>) -> Self {
        Self(self.0 + other.0)
    }
}

// R += R
impl<T: Float> AddAssign for ScalarWrapper<T> {
    fn add_assign(&mut self, other: ScalarWrapper<T>) {
        *self = Self(self.0 + other.0)
    }
}

// R - R
impl<T: Float> Sub for ScalarWrapper<T> {
    type Output = Self;
    fn sub(self, other: ScalarWrapper<T>) -> Self {
        Self(self.0 - other.0)
    }
}

// -R
impl<T: Float> Neg for ScalarWrapper<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

// R -= R
impl<T: Float> SubAssign for ScalarWrapper<T> {
    fn sub_assign(&mut self, other: ScalarWrapper<T>) {
        *self = Self(self.0 - other.0)
    }
}

// R * R
impl<T: Float> Mul for ScalarWrapper<T> {
    type Output = Self;
    fn mul(self, other: ScalarWrapper<T>) -> Self {
        Self(self.0 * other.0)
    }
}

// R *= R
impl<T: Float> MulAssign for ScalarWrapper<T> {
    fn mul_assign(&mut self, other: ScalarWrapper<T>) {
        *self = Self(self.0 * other.0)
    }
}

// R + R(Float)
impl<T: Float> Add<T> for ScalarWrapper<T> {
    type Output = Self;
    fn add(self, other: T) -> Self {
        Self(self.0 + other)
    }
}

// R - R(Float)
impl<T: Float> Sub<T> for ScalarWrapper<T> {
    type Output = Self;
    fn sub(self, other: T) -> Self {
        Self(self.0 - other)
    }
}

// R * R(Float)
impl<T: Float> Mul<T> for ScalarWrapper<T> {
    type Output = Self;
    fn mul(self, other: T) -> Self {
        Self(self.0 * other)
    }
}

// R + H
impl<T: Float> Add<QuaternionWrapper<T>> for ScalarWrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn add(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper(( self.0 + (other.0).0, (other.0).1 ))
    }
}

// R - H
impl<T: Float> Sub<QuaternionWrapper<T>> for ScalarWrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn sub(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper(( self.0 - (other.0).0, quat::negate_vec((other.0).1) ))
    }
}

// R * H
impl<T: FloatSimd<T>> Mul<QuaternionWrapper<T>> for ScalarWrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn mul(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper( quat::scale(self.0, other.0) )
    }
}

// R + R^3
impl<T: Float> Add<Vector3Wrapper<T>> for ScalarWrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn add(self, other: Vector3Wrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper( (self.0, other.0) )
    }
}

// R - R^3
impl<T: Float> Sub<Vector3Wrapper<T>> for ScalarWrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn sub(self, other: Vector3Wrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper( (self.0, quat::negate_vec(other.0)) )
    }
}

// R * R^3
impl<T: Float> Mul<Vector3Wrapper<T>> for ScalarWrapper<T> {
    type Output = Vector3Wrapper<T>;
    fn mul(self, other: Vector3Wrapper<T>) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::scale_vec(self.0, other.0) )
    }
}
