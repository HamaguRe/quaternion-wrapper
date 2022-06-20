//! 演算子オーバーロードを実装

use core::ops::{Add, AddAssign, Sub, SubAssign, Neg, Mul, MulAssign, Div, DivAssign};
use super::{quat, QuaternionWrapper, Vector3Wrapper, ScalarWrapper};
use super::Float;


// ------------------------ Quaternion ------------------------ //
// H + H
impl<T: Float> Add for QuaternionWrapper<T> {
    type Output = Self;
    fn add(self, other: QuaternionWrapper<T>) -> Self {
        Self( quat::add(self.0, other.0) )
    }
}

// H += H
impl<T: Float + Copy> AddAssign for QuaternionWrapper<T> {
    fn add_assign(&mut self, other: QuaternionWrapper<T>) {
        *self = Self( quat::add(self.0, other.0) );
    }
}

// H + R^3
impl<T: Float> Add<Vector3Wrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn add(self, other: Vector3Wrapper<T>) -> Self {
        Self(( (self.0).0, quat::add((self.0).1, other.0) ))
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
impl<T: Float> Sub for QuaternionWrapper<T> {
    type Output = Self;
    fn sub(self, other: QuaternionWrapper<T>) -> Self {
        Self( quat::sub(self.0, other.0) )
    }
}

// H -= H
impl<T: Float + Copy> SubAssign for QuaternionWrapper<T> {
    fn sub_assign(&mut self, other: QuaternionWrapper<T>) {
        *self = Self( quat::sub(self.0, other.0) );
    }
}

// -H
impl<T: Float> Neg for QuaternionWrapper<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Self( quat::negate(self.0) )
    }
}

// H - R^3
impl<T: Float> Sub<Vector3Wrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn sub(self, other: Vector3Wrapper<T>) -> Self {
        Self(( (self.0).0, quat::sub((self.0).1, other.0) ))
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
impl<T: Float> Mul for QuaternionWrapper<T> {
    type Output = Self;
    fn mul(self, other: QuaternionWrapper<T>) -> Self {
        Self( quat::mul(self.0, other.0) )
    }
}

// H *= H
impl<T: Float> MulAssign for QuaternionWrapper<T> {
    fn mul_assign(&mut self, other: QuaternionWrapper<T>) {
        *self = Self( quat::mul(self.0, other.0) );
    }
}

// H * R^3
impl<T: Float> Mul<Vector3Wrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn mul(self, other: Vector3Wrapper<T>) -> Self {
        Self((
            -quat::dot((self.0).1, other.0), 
            quat::scale_add((self.0).0, other.0, quat::cross((self.0).1, other.0)) 
        ))
    }
}

// H * R
impl<T: Float> Mul<ScalarWrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn mul(self, other: ScalarWrapper<T>) -> Self {
        Self( quat::scale(other.0, self.0) )
    }
}

// H / R
impl<T: Float> Div<ScalarWrapper<T>> for QuaternionWrapper<T> {
    type Output = Self;
    fn div(self, other: ScalarWrapper<T>) -> Self {
        Self( quat::scale(other.0.recip(), self.0) )
    }
}

// -------------------------- Vector3 ------------------------- //
// R^3 + R^3
impl<T: Float> Add for Vector3Wrapper<T> {
    type Output = Self;
    fn add(self, other: Vector3Wrapper<T>) -> Self {
        Self( quat::add(self.0, other.0) )
    }
}

// R^3 += R^3
impl<T: Float> AddAssign for Vector3Wrapper<T> {
    fn add_assign(&mut self, other: Vector3Wrapper<T>) {
        *self = Self( quat::add(self.0, other.0) )
    }
}

// R^3 + H
impl<T: Float> Add<QuaternionWrapper<T>> for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn add(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper(( (other.0).0, quat::add(self.0, (other.0).1) ))
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
        Self( quat::sub(self.0, other.0) )
    }
}

// R^3 -= R^3
impl<T: Float> SubAssign for Vector3Wrapper<T> {
    fn sub_assign(&mut self, other: Vector3Wrapper<T>) {
        *self = Self( quat::sub(self.0, other.0) )
    }
}

// -R
impl<T: Float> Neg for Vector3Wrapper<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Self( quat::negate(self.0) )
    }
}

// R^3 - H
impl<T: Float> Sub<QuaternionWrapper<T>> for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn sub(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper(( -(other.0).0, quat::sub(self.0, (other.0).1) ))
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
        QuaternionWrapper( quat::mul(self.0, other.0) )
    }
}

// R^3 * H
impl<T: Float> Mul<QuaternionWrapper<T>> for Vector3Wrapper<T> {
    type Output = QuaternionWrapper<T>;
    fn mul(self, other: QuaternionWrapper<T>) -> QuaternionWrapper<T> {
        QuaternionWrapper((
            -quat::dot(self.0, (other.0).1),
            quat::scale_add( (other.0).0, self.0, quat::cross(self.0, (other.0).1) )
        ))
    }
}

// R^3 * R
impl<T: Float> Mul<ScalarWrapper<T>> for Vector3Wrapper<T> {
    type Output = Self;
    fn mul(self, other: ScalarWrapper<T>) -> Self {
        Self( quat::scale(other.0, self.0) )
    }
}

// R^3 / R
impl<T: Float> Div<ScalarWrapper<T>> for Vector3Wrapper<T> {
    type Output = Self;
    fn div(self, other: ScalarWrapper<T>) -> Self {
        Self( quat::scale(other.0.recip(), self.0) )
    }
}

// --------------------------- Scalar ------------------------- //
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

// R / R
impl<T: Float> Div for ScalarWrapper<T> {
    type Output = Self;
    fn div(self, other: ScalarWrapper<T>) -> Self {
        Self(self.0 / other.0)
    }
}

// R /= R
impl<T: Float> DivAssign for ScalarWrapper<T> {
    fn div_assign(&mut self, other: ScalarWrapper<T>) {
        *self = Self(self.0 / other.0)
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
        QuaternionWrapper(( self.0 - (other.0).0, quat::negate((other.0).1) ))
    }
}

// R * H
impl<T: Float> Mul<QuaternionWrapper<T>> for ScalarWrapper<T> {
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
        QuaternionWrapper( (self.0, quat::negate(other.0)) )
    }
}

// R * R^3
impl<T: Float> Mul<Vector3Wrapper<T>> for ScalarWrapper<T> {
    type Output = Vector3Wrapper<T>;
    fn mul(self, other: Vector3Wrapper<T>) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::scale(self.0, other.0) )
    }
}
