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
pub use quat::{Axis, Vector3, Quaternion, DCM, RotationType, RotationSequence};

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

impl<T> QuaternionWrapper<T> {
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

    /// Returns the `Quaternion<T>`.
    #[inline]
    pub fn unwrap(self) -> Quaternion<T> {
        self.0
    }

    /// Returns the scalar part as a `ScalarWrapper<T>`.
    #[inline]
    pub fn get_scalar_part(self) -> ScalarWrapper<T> {
        ScalarWrapper( (self.0).0 )
    }

    /// Returns the vector part as a `Vector3Wrapper<T>`.
    #[inline]
    pub fn get_vector_part(self) -> Vector3Wrapper<T> {
        Vector3Wrapper( (self.0).1 )
    }
}

impl<T> Vector3Wrapper<T> {
    /// Create a new `Vector3Wrapper` from a `Vector3<T>`.
    #[inline]
    pub fn new(v: Vector3<T>) -> Self {
        Self(v)
    }

    /// Returns the inner `Vector3<T>`.
    #[inline]
    pub fn unwrap(self) -> Vector3<T> {
        self.0
    }
}

impl<T> ScalarWrapper<T> {
    /// Create a new `ScalarWrapper`.
    #[inline]
    pub fn new(s: T) -> Self {
        Self(s)
    }

    /// Returns the inner value of type `T`.
    #[inline]
    pub fn unwrap(self) -> T {
        self.0
    }
}

// ------------------------ Quaternion ------------------------ //
impl<T: Float> QuaternionWrapper<T> {
    /// Generate an identity Quaternion.
    ///
    /// Its inner Quaternion is `(1.0, [0.0, 0.0, 0.0])`.
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
        Self( quat::identity() )
    }

    /// Converts a Quaternion between right-handed and left-handed coordinate systems.
    ///
    /// The two systems are mirror images of each other: one coordinate axis points
    /// the opposite way, and the positive direction of rotation is reversed as well.
    /// This method accounts for both, returning the `QuaternionWrapper` that describes the
    /// same rotation in the other coordinate system.
    ///
    /// The `axis` argument specifies which coordinate axis is reversed between the
    /// source and destination coordinate systems. For example, pass `Axis::Z` when
    /// the Z-axis is reversed between the two systems.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{Axis, QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Rotation of pi/2[rad] around the axis (1, 1, 1), described in a right-handed system.
    /// let q_rh = QuaternionWrapper::from_axis_angle(Vector3Wrapper([1.0, 1.0, 1.0]), PI/2.0);
    ///
    /// // The same rotation, described in a left-handed system
    /// // whose z-axis points the opposite way.
    /// let q_lh = q_rh.convert_handedness(Axis::Z);
    ///
    /// // In the mirrored system, this rotation is described as -pi/2[rad]
    /// // around the axis (1, 1, -1).
    /// let expected = QuaternionWrapper::from_axis_angle(Vector3Wrapper([1.0, 1.0, -1.0]), -PI/2.0);
    /// let diff = (q_lh - expected).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    ///
    /// // Applying the conversion twice restores the original Quaternion.
    /// let diff = (q_rh - q_lh.convert_handedness(Axis::Z)).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    /// ```
    #[inline]
    pub fn convert_handedness(self, axis: Axis) -> Self {
        Self( quat::convert_handedness(self.0, axis) )
    }

    /// Generate a Versor by specifying rotation `angle`\[rad\] and `axis` vector.
    /// 
    /// The `axis` vector does not have to be a unit vector.
    /// 
    /// If you enter a zero vector, it returns an identity `QuaternionWrapper`.
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Generates a quaternion representing the
    /// // rotation of π/2[rad] around the y-axis.
    /// let q = QuaternionWrapper::from_axis_angle(Vector3Wrapper([0.0, 1.0, 0.0]), PI/2.0);
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
    pub fn from_axis_angle(axis: Vector3Wrapper<T>, angle: T) -> Self
    where T: FloatConst {
        Self( quat::from_axis_angle(axis.0, angle) )
    }

    /// Converts a **Direction Cosine Matrix (DCM)** into a **Versor**.
    ///
    /// Crucially, this method assumes the input DCM represents a
    /// **Point Rotation (Frame Fixed)**, where a vector `v` is rotated by `q v q*`.
    ///
    /// If the input DCM represents a **Frame Rotation (Point Fixed)**,
    /// which corresponds to `q* v q`, take the conjugate of the resulting
    /// `QuaternionWrapper`:
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
    /// let q = QuaternionWrapper::from_axis_angle(Vector3Wrapper([0.2, 1.0, -2.0]), PI/4.0);
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

    /// Converts **Euler Angles** into a **Versor**.
    ///
    /// This method requires two parameters to fully define the rotation:
    ///
    /// 1. `RotationType`: Specifies whether the rotation is **Intrinsic** or **Extrinsic**.
    /// 2. `RotationSequence`: Defines the three-axis sequence (e.g., XYZ, ZYX, XZX, ...).
    ///
    /// The input `angles` array must contain the three angles in **radians**,
    /// corresponding to the specified rotation sequence: `angles[0]` -> `angles[1]` -> `angles[2]`.
    /// Each angle should be within the range `[-2*PI, 2*PI]`.
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// use quaternion_wrapper::{RotationType::*, RotationSequence::XYZ};
    /// 
    /// let angles = Vector3Wrapper([PI/6.0, 1.6*PI, -PI/4.0]);
    /// let v = Vector3Wrapper([1.0, 0.5, -0.4]);
    /// 
    /// // Quaternions representing rotation around each axis.
    /// let x = QuaternionWrapper::from_axis_angle(Vector3Wrapper([1.0, 0.0, 0.0]), angles.0[0]);
    /// let y = QuaternionWrapper::from_axis_angle(Vector3Wrapper([0.0, 1.0, 0.0]), angles.0[1]);
    /// let z = QuaternionWrapper::from_axis_angle(Vector3Wrapper([0.0, 0.0, 1.0]), angles.0[2]);
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
    pub fn from_euler_angles(rt: RotationType, rs: RotationSequence, angles: Vector3Wrapper<T>) -> Self {
        Self( quat::from_euler_angles(rt, rs, angles.0) )
    }

    /// Extracts the rotation `axis` and the rotation `angle` around the `axis` from the Versor.
    ///
    /// # Returns
    ///
    /// The method returns a tuple `(axis, angle)`, where:
    ///
    /// * **`axis`**: The rotation axis as a **unit vector** (`Vector3Wrapper`).
    /// * **`angle`**: The rotation angle in **radians**. The range is `(-PI, PI]`.
    ///
    /// ## Special Case: Identity Quaternion
    ///
    /// If the input is the **Identity Quaternion** `(1.0, [0.0, 0.0, 0.0])`,
    /// the method returns an angle of zero and a zero axis vector.
    ///
    /// # Singularity
    ///
    /// Because this method returns a normalized rotation axis,
    /// when the norm of the vector part of Versor is zero, a singularity
    /// occurs and accuracy decreases.
    ///
    /// If you want to use the calculated `angle` and `axis` as `axis * angle`,
    /// it is better to use the `to_rotation_vector` method.
    /// The `to_rotation_vector` method can be calculated without singularities.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// let axis_original = Vector3Wrapper([0.0_f64, 1.0, 2.0]);
    /// let angle_original = PI / 2.0;
    /// let q = QuaternionWrapper::from_axis_angle(axis_original, angle_original);
    /// let (axis, angle) = q.to_axis_angle();
    /// let diff = (axis_original.normalize() - axis.normalize()).unwrap();
    /// assert!(diff.iter().all(|value| value.abs() < 1e-12));
    /// assert!((angle_original - angle.unwrap()).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn to_axis_angle(self) -> (Vector3Wrapper<T>, ScalarWrapper<T>) {
        let f = quat::to_axis_angle(self.0);
        ( Vector3Wrapper(f.0), ScalarWrapper(f.1) )
    }

    /// Converts a **Versor** into a **Direction Cosine Matrix (DCM)**.
    ///
    /// **By default, the output DCM represents a Point Rotation (Frame Fixed)**,
    /// which rotates a vector `v` by the quaternion operation `q v q*`.
    ///
    /// If you need a DCM that represents a **Frame Rotation (Point Fixed)**
    /// (the rotation `q* v q`), take the conjugate of the Versor:
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
    /// let q = QuaternionWrapper::from_axis_angle(Vector3Wrapper([0.2, 1.0, -2.0]), PI/4.0);
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

    /// Converts a **Versor** into **Euler Angles**.
    ///
    /// This method requires two parameters to fully define the rotation:
    ///
    /// 1. `RotationType`: Specifies whether the rotation is **Intrinsic** or **Extrinsic**.
    /// 2. `RotationSequence`: Defines the three-axis sequence (e.g., XYZ, ZYX, XZX, ...).
    ///
    /// The output `Vector3Wrapper` contains the three angles corresponding to the sequence:
    /// `angles[0]` -> `angles[1]` -> `angles[2]`.
    /// Each angle is returned in the range `(-PI, PI]`.
    ///
    /// # Singularity (Gimbal Lock)
    /// 
    /// ## RotationType::Intrinsic
    /// 
    /// For Proper Euler angles (ZXZ, XYX, YZY, ZYZ, XZX, YXY), the singularity is reached 
    /// when the sine of the second rotation angle is 0 (angle = 0, ±π, ...), and for 
    /// Tait-Bryan angles (XYZ, YZX, ZXY, XZY, ZYX, YXZ), the singularity is reached when 
    /// the cosine of the second rotation angle is 0 (angle = ±π/2).
    /// 
    /// ## RotationType::Extrinsic
    /// 
    /// As in the case of Intrinsic rotation, for Proper Euler angles, the singularity occurs 
    /// when the sine of the second rotation angle is 0 (angle = 0, ±π, ...), and for 
    /// Tait-Bryan angles, the singularity occurs when the cosine of the second rotation angle 
    /// is 0 (angle = ±π/2).
    /// 
    /// ## Resolution at Singularity
    ///
    /// * For **Intrinsic** rotation, the **third angle** (`angles[2]`) is set to 0 \[rad\].
    /// * For **Extrinsic** rotation, the **first angle** (`angles[0]`) is set to 0 \[rad\].
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
    /// let q_in = QuaternionWrapper::from_euler_angles(Intrinsic, XYZ, angles);
    /// let e_in = q_in.to_euler_angles(Intrinsic, XYZ);
    /// let diff = (angles - e_in).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    /// 
    /// // ---- Extrinsic (X-Y-Z) ---- //
    /// let q_ex = QuaternionWrapper::from_euler_angles(Extrinsic, XYZ, angles);
    /// let e_ex = q_ex.to_euler_angles(Extrinsic, XYZ);
    /// let diff = (angles - e_ex).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn to_euler_angles(self, rt: RotationType, rs: RotationSequence) -> Vector3Wrapper<T>
    where T: FloatConst {
        Vector3Wrapper( quat::to_euler_angles(rt, rs, self.0) )
    }

    /// Converts a **Rotation Vector** into a **Versor**.
    ///
    /// A Rotation Vector is a convenient representation where:
    ///
    /// 1. Its **direction** defines the **rotation axis**.
    /// 2. Its **norm** defines the **rotation angle** (in radians).
    ///
    /// There are no particular restrictions on the norm of the input Rotation Vector.
    /// Also, even if a zero vector is input, the conversion to a Versor can be performed
    /// without falling into a singularity.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper, ScalarWrapper};
    /// # let PI = std::f64::consts::PI;
    /// let angle = ScalarWrapper(PI / 2.0);
    /// let axis = Vector3Wrapper([1.0, 0.0, 0.0]);
    ///
    /// // This represents a rotation of π/2 around the x-axis.
    /// let rot_vec = axis * angle;  // Rotation vector
    ///
    /// // Rotation vector ---> Quaternion
    /// let q = QuaternionWrapper::from_rotation_vector(rot_vec);
    ///
    /// let r = q.point_rotation( Vector3Wrapper([1.0, 1.0, 0.0]) ).unwrap();
    ///
    /// assert!( (r[0] - 1.0).abs() < 1e-12 );
    /// assert!( (r[1] - 0.0).abs() < 1e-12 );
    /// assert!( (r[2] - 1.0).abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn from_rotation_vector(r: Vector3Wrapper<T>) -> Self
    where T: FloatConst {
        Self( quat::from_rotation_vector(r.0) )
    }

    /// Converts a **Versor** into a **Rotation Vector**.
    ///
    /// A Rotation Vector is a convenient representation where:
    ///
    /// 1. Its **direction** defines the **rotation axis**.
    /// 2. Its **norm** defines the **rotation angle** (in radians).
    ///
    /// The resulting `Vector3Wrapper`'s norm (the rotation angle) is always constrained
    /// to the range: `[0, PI]`
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper, ScalarWrapper};
    /// # let PI = std::f64::consts::PI;
    /// let angle = PI / 2.0;
    /// let axis = Vector3Wrapper([1.0, 0.0, 0.0]);
    ///
    /// // These represent the same rotation.
    /// let rv = axis * ScalarWrapper(angle);  // Rotation vector
    /// let q = QuaternionWrapper::from_axis_angle(axis, angle);  // Quaternion
    ///
    /// // Quaternion ---> Rotation vector
    /// let diff = (rv - q.to_rotation_vector()).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn to_rotation_vector(self) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::to_rotation_vector(self.0) )
    }

    /// Calculate the Versor to rotate from vector `a` to vector `b` (Without singularity!).
    ///
    /// This method calculates `q` satisfying `b = q.point_rotation(a)`
    /// when `a.norm() = b.norm()`.
    ///
    /// # Characteristics
    ///
    /// * **Robustness:** This method can accurately calculate the Versor regardless of the
    ///   combination of vector orientations (however, `a.norm() > 0` and `b.norm() > 0`).
    /// * **Axis Ambiguity:** This method provides **no guarantees** regarding the direction
    ///   or regularity of the rotation axis. If you require a rotation axis that is **orthogonal**
    ///   to both vector `a` and vector `b`, use the `rotate_a_to_b_shortest` method.
    ///
    /// # Returns
    ///
    /// Returns `None` if either input vector `a` or `b` is a zero vector,
    /// as a rotation cannot be uniquely defined in that case.
    ///
    /// # Example
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// let a = Vector3Wrapper::<f64>::new([1.5, -0.5, 0.2]);
    /// let b = Vector3Wrapper::<f64>::new([0.1, 0.6, 1.0]);
    /// 
    /// let q = QuaternionWrapper::rotate_a_to_b(a, b).unwrap();
    /// let b_check = q.point_rotation(a);
    /// 
    /// let diff = (b.normalize() - b_check.normalize()).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn rotate_a_to_b(a: Vector3Wrapper<T>, b: Vector3Wrapper<T>) -> Option<Self> {
        quat::rotate_a_to_b(a.0, b.0).map(Self)
    }

    /// Calculate the Versor to rotate from vector `a` to vector `b` by the shortest path.
    ///
    /// This method calculates `q` satisfying `b = q.point_rotation(a)`
    /// when `a.norm() = b.norm()`.
    ///
    /// # Characteristics
    ///
    /// * **Shortest Path:** This method guarantees the rotation axis is always **orthogonal**
    ///   to both vector `a` and vector `b`, representing the geometrically shortest path between
    ///   the two directions.
    /// * **Rotation Angle:** The rotation angle is in the range `[0, PI]` radians.
    /// * **Parallel Vectors:** If vector `a` and `b` are **opposite** and parallel, the
    ///   rotation axis is theoretically ambiguous. This method provides a valid rotation,
    ///   but the axis direction is not guaranteed (although it will be orthogonal to the `a` and `b`).
    ///
    /// # Performance Consideration
    ///
    /// This method is slightly more computationally intensive than `rotate_a_to_b`,
    /// especially when the angle between the vectors is near PI. If an orthogonal rotation
    /// axis is not strictly required, `rotate_a_to_b` may be preferred for performance.
    ///
    /// # Returns
    ///
    /// Returns `None` if either input vector `a` or `b` is a zero vector,
    /// as a rotation cannot be uniquely defined in that case.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper, ScalarWrapper};
    /// let a = Vector3Wrapper::<f64>::new([1.5, -0.5, 0.2]);
    /// let b = Vector3Wrapper::<f64>::new([0.1, 0.6, 1.0]);
    ///
    /// let q = QuaternionWrapper::rotate_a_to_b_shortest(a, b).unwrap();
    /// let b_check = q.point_rotation(a);
    ///
    /// let diff = (b.normalize() - b_check.normalize()).unwrap();
    /// assert!( diff[0].abs() < 1e-12 );
    /// assert!( diff[1].abs() < 1e-12 );
    /// assert!( diff[2].abs() < 1e-12 );
    ///
    /// // --- If the amount of displacement of vector `a` is to be adjusted --- //
    /// // The parameter `t` adjusts the amount of movement from `a` to `b`.
    /// // When `t = 1`, `a` moves completely to position `b`.
    /// let t = ScalarWrapper(0.5);
    /// let q = QuaternionWrapper::rotate_a_to_b_shortest(a, b).unwrap();
    /// let r = q.to_rotation_vector();  // To avoid singularities, proceed via the rotation vector.
    /// let q = QuaternionWrapper::from_rotation_vector(r * t);
    /// ```
    #[inline]
    pub fn rotate_a_to_b_shortest(a: Vector3Wrapper<T>, b: Vector3Wrapper<T>) -> Option<Self> {
        quat::rotate_a_to_b_shortest(a.0, b.0).map(Self)
    }

    /// Sum all elements of the Quaternion and return the result as a `ScalarWrapper`.
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

    /// Scaling and addition in one step: `s * self + b`.
    /// 
    /// If the `fma` feature is enabled, the FMA calculation is performed using
    /// the scalar type's `mul_add` method for each component.
    /// If not enabled, it is computed by unfused multiply-add (`s * self + b`).
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, ScalarWrapper};
    /// let q1 = QuaternionWrapper((1.0_f64, [2.0, 3.0, 4.0]));
    /// let q2 = QuaternionWrapper((0.1, [0.2, 0.3, 0.4]));
    /// let result = q1.scale_add(ScalarWrapper(2.0), q2).unwrap();
    /// assert!((result.0 - 2.1).abs() < 1e-12);
    /// assert!((result.1[0] - 4.2).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn scale_add(self, s: ScalarWrapper<T>, b: QuaternionWrapper<T>) -> Self {
        Self( quat::scale_add(s.0, self.0, b.0) )
    }

    /// Calculate the element-wise product of two Quaternions: `self ∘ other`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q1 = QuaternionWrapper((1.0_f64, [2.0, 3.0, 4.0]));
    /// let q2 = QuaternionWrapper((0.1, [0.2, 0.3, 0.4]));
    /// let result = q1.hadamard(q2).unwrap();
    /// assert!((result.0 - 0.1).abs() < 1e-12);
    /// assert!((result.1[2] - 1.6).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn hadamard(self, other: QuaternionWrapper<T>) -> Self {
        Self( quat::hadamard(self.0, other.0) )
    }

    /// Hadamard product and addition in one step: `self ∘ b + c`.
    /// 
    /// If the `fma` feature is enabled, the FMA calculation is performed using
    /// the `mul_add` method.
    /// If not enabled, it is computed by unfused multiply-add for each component
    /// (`self ∘ b + c`).
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q1 = QuaternionWrapper((1.0_f64, [2.0, 3.0, 4.0]));
    /// let q2 = QuaternionWrapper((0.1, [0.2, 0.3, 0.4]));
    /// let q3 = QuaternionWrapper((0.5, [0.6, 0.7, 0.8]));
    /// let result = q1.hadamard_add(q2, q3).unwrap();
    /// assert!((result.0 - 0.6).abs() < 1e-12);
    /// assert!((result.1[2] - 2.4).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn hadamard_add(self, b: QuaternionWrapper<T>, c: QuaternionWrapper<T>) -> Self {
        Self( quat::hadamard_add(self.0, b.0, c.0) )
    }

    /// Dot product of two Quaternions: `self · other`.
    ///
    /// The result is returned as a `ScalarWrapper`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q1 = QuaternionWrapper((1.0_f64, [2.0, 3.0, 4.0]));
    /// let q2 = QuaternionWrapper((0.1, [0.2, 0.3, 0.4]));
    /// assert!((q1.dot(q2).unwrap() - 3.0).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn dot(self, other: QuaternionWrapper<T>) -> ScalarWrapper<T> {
        ScalarWrapper( quat::dot(self.0, other.0) )
    }

    /// Calculate the L2 norm of the Quaternion and return it as a `ScalarWrapper`.
    ///
    /// Compared to `self.dot(self).unwrap().sqrt()`, this method is less likely
    /// to cause overflow and underflow.
    ///
    /// When the `norm-sqrt` feature is enabled, the default
    /// implementation is replaced with `self.dot(self).unwrap().sqrt()` internally.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper((1.0_f64, [2.0, 3.0, 4.0]));
    /// assert!((q.norm().unwrap() - 30.0_f64.sqrt()).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn norm(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::norm(self.0) )
    }

    /// Normalize the Quaternion.
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

    /// Calculate the conjugate of the Quaternion.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper((1.0_f64, [2.0, 3.0, 4.0]));
    /// assert_eq!(q.conj().unwrap(), (1.0, [-2.0, -3.0, -4.0]));
    /// ```
    #[inline]
    pub fn conj(self) -> Self {
        Self( quat::conj(self.0) )
    }

    /// Calculate the inverse of the Quaternion.
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

    /// Exponential function of the Quaternion.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, ScalarWrapper};
    /// let q = QuaternionWrapper((0.1_f64, [0.2, 0.3, 0.4]));
    /// let q_r = q.exp().ln();
    /// let diff = (q - q_r).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    ///
    /// // The relationship between exp(q) and the exponential of its vector part.
    /// let expected = q.get_vector_part().exp() * ScalarWrapper(q.get_scalar_part().unwrap().exp());
    /// let diff = (q.exp() - expected).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    /// ```
    #[inline]
    pub fn exp(self) -> Self {
        Self( quat::exp(self.0) )
    }

    /// Natural logarithm of the Quaternion.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper((0.1_f64, [0.2, 0.3, 0.4]));
    /// let diff = (q - q.exp().ln()).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    /// ```
    #[inline]
    pub fn ln(self) -> Self {
        Self( quat::ln(self.0) )
    }

    /// Natural logarithm of the Versor.
    /// 
    /// If `self` is guaranteed to be a Versor, this method is less
    /// computationally expensive than `.ln()`.
    /// 
    /// Only the vector part is returned since the real part is always zero.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// let v = Vector3Wrapper([0.1_f64, 0.2, 0.3]);
    /// let diff = (v - v.exp().ln_versor()).unwrap();
    /// assert!(diff.iter().all(|value| value.abs() < 1e-12));
    /// ```
    #[inline]
    pub fn ln_versor(self) -> Vector3Wrapper<T> {
        Vector3Wrapper( quat::ln_versor(self.0) )
    }

    /// Power function of the Quaternion.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper((1.0_f64, [2.0, 3.0, 4.0]));
    /// assert_eq!(q.pow(0.0).unwrap(), (1.0, [0.0; 3]));
    /// let diff = (q * q - q.pow(2.0)).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    /// let diff = (q.sqrt() - q.pow(0.5)).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    /// ```
    #[inline]
    pub fn pow(self, t: T) -> Self {
        Self( quat::pow(self.0, t) )
    }

    /// Power function of the Versor.
    /// 
    /// If `self` is guaranteed to be a Versor, this method is less
    /// computationally expensive than `.pow()`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper((1.0_f64, [2.0, 3.0, 4.0])).normalize();
    /// assert_eq!(q.pow_versor(0.0).unwrap(), (1.0, [0.0; 3]));
    /// let diff = (q * q - q.pow_versor(2.0)).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    /// let diff = (q.sqrt() - q.pow_versor(0.5)).unwrap();
    /// assert!(diff.0.abs() < 1e-12);
    /// assert!(diff.1.iter().all(|value| value.abs() < 1e-12));
    /// ```
    #[inline]
    pub fn pow_versor(self, t: T) -> Self {
        Self( quat::pow_versor(self.0, t) )
    }

    /// Square root of the Quaternion.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper::<f64>::new( (1.0, [2.0, 3.0, 4.0]) );
    /// let q_sqrt = q.sqrt();
    ///
    /// let diff = (q - q_sqrt * q_sqrt).unwrap();
    /// assert!( diff.0.abs() < 1e-12 );
    /// assert!( diff.1[0].abs() < 1e-12 );
    /// assert!( diff.1[1].abs() < 1e-12 );
    /// assert!( diff.1[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn sqrt(self) -> Self {
        Self( quat::sqrt(self.0) )
    }

    /// Square root of the Versor.
    ///
    /// If `self` is guaranteed to be a Versor, this method is less
    /// computationally expensive than `.sqrt()`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::QuaternionWrapper;
    /// let q = QuaternionWrapper::<f64>::new( (1.0, [2.0, 3.0, 4.0]) ).normalize();
    /// let q_sqrt = q.sqrt_versor();
    ///
    /// let diff = (q - q_sqrt * q_sqrt).unwrap();
    /// assert!( diff.0.abs() < 1e-12 );
    /// assert!( diff.1[0].abs() < 1e-12 );
    /// assert!( diff.1[1].abs() < 1e-12 );
    /// assert!( diff.1[2].abs() < 1e-12 );
    /// ```
    #[inline]
    pub fn sqrt_versor(self) -> Self {
        Self( quat::sqrt_versor(self.0) )
    }

    /// Rotates a point by the Versor (Point Rotation - Frame Fixed).
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
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Make these as you like.
    /// let v = Vector3Wrapper([1.0, 0.5, -8.0]);
    /// let q = QuaternionWrapper::from_axis_angle(Vector3Wrapper([0.2, 1.0, -2.0]), PI);
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

    /// Rotates a frame by the Versor (Frame Rotation - Point Fixed).
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
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::{QuaternionWrapper, Vector3Wrapper};
    /// # let PI = std::f64::consts::PI;
    /// // Make these as you like.
    /// let v = Vector3Wrapper([1.0, 0.5, -8.0]);
    /// let q = QuaternionWrapper::from_axis_angle(Vector3Wrapper([0.2, 1.0, -2.0]), PI);
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
    /// Generate a Versor that interpolates the shortest path from `self` to `other`.
    /// The argument `t (0 <= t <= 1)` is the interpolation parameter.
    /// 
    /// `self` and `other` must be Versors.
    ///
    /// Normalization is not performed internally because 
    /// it increases the computational complexity.
    #[inline]
    pub fn lerp(self, other: QuaternionWrapper<T>, t: T) -> Self {
        Self( quat::lerp(self.0, other.0, t) )
    }

    /// Slerp (Spherical linear interpolation)
    /// 
    /// Generate a Versor that interpolates the shortest path from `self` to `other`.
    /// The argument `t(0 <= t <= 1)` is the interpolation parameter.
    /// 
    /// `self` and `other` must be Versors.
    #[inline]
    pub fn slerp(self, other: QuaternionWrapper<T>, t: T) -> Self {
        Self( quat::slerp(self.0, other.0, t) )
    }
}

// ------------------------- Vector3 ------------------------- //
impl<T: Float> Vector3Wrapper<T> {
    /// Product of a **Direction Cosine Matrix (DCM)** and this **Vector3**.
    ///
    /// This computes the mathematical product with `m` on the left and `self` on the right.
    /// It is the product of a 3x3 matrix and a 3D vector.
    /// It is used to apply the rotation represented by the DCM to the vector.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// # let PI = std::f64::consts::PI;
    /// let theta = PI / 2.0;
    /// let rot_x = [
    ///     [1.0, 0.0, 0.0],
    ///     [0.0, theta.cos(), -theta.sin()],
    ///     [0.0, theta.sin(), theta.cos()],
    /// ];
    /// let result = Vector3Wrapper([0.0, 1.0, 0.0]).matrix_product(rot_x).unwrap();
    /// assert!((result[0] - 0.0).abs() < 1e-12);
    /// assert!((result[1] - 0.0).abs() < 1e-12);
    /// assert!((result[2] - 1.0).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn matrix_product(self, m: DCM<T>) -> Self {
        Self( quat::matrix_product(m, self.0) )
    }

    /// Sum all elements of the Vector3 and return the result as a `ScalarWrapper`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// let v = Vector3Wrapper([1.0_f64, 2.0, 3.0]);
    /// assert!((v.sum().unwrap() - 6.0).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn sum(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::sum(self.0) )
    }

    /// Scaling and addition in one step: `s * self + b`.
    /// 
    /// If the `fma` feature is enabled, the FMA calculation is performed using
    /// the scalar type's `mul_add` method for each component.
    /// If not enabled, it is computed by unfused multiply-add (`s * self + b`).
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{ScalarWrapper, Vector3Wrapper};
    /// let v1 = Vector3Wrapper([1.0_f64, 2.0, 3.0]);
    /// let v2 = Vector3Wrapper([0.1, 0.2, 0.3]);
    /// let result = v1.scale_add(ScalarWrapper(2.0), v2).unwrap();
    /// assert!((result[0] - 2.1).abs() < 1e-12);
    /// assert!((result[2] - 6.3).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn scale_add(self, s: ScalarWrapper<T>, b: Vector3Wrapper<T>) -> Self {
        Self( quat::scale_add(s.0, self.0, b.0) )
    }

    /// Calculate the element-wise product of two Vector3s: `self ∘ other`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// let v1 = Vector3Wrapper([1.0_f64, 2.0, 3.0]);
    /// let v2 = Vector3Wrapper([0.1, 0.2, 0.3]);
    /// let result = v1.hadamard(v2).unwrap();
    /// assert!((result[0] - 0.1).abs() < 1e-12);
    /// assert!((result[2] - 0.9).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn hadamard(self, other: Vector3Wrapper<T>) -> Self {
        Self( quat::hadamard(self.0, other.0) )
    }

    /// Hadamard product and addition in one step: `self ∘ b + c`.
    /// 
    /// If the `fma` feature is enabled, the FMA calculation is performed using
    /// the `mul_add` method.
    /// If not enabled, it is computed by unfused multiply-add for each component
    /// (`self ∘ b + c`).
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// let v1 = Vector3Wrapper([1.0_f64, 2.0, 3.0]);
    /// let v2 = Vector3Wrapper([0.1, 0.2, 0.3]);
    /// let v3 = Vector3Wrapper([0.4, 0.5, 0.6]);
    /// let result = v1.hadamard_add(v2, v3).unwrap();
    /// assert!((result[0] - 0.5).abs() < 1e-12);
    /// assert!((result[2] - 1.5).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn hadamard_add(self, b: Vector3Wrapper<T>, c: Vector3Wrapper<T>) -> Self {
        Self( quat::hadamard_add(self.0, b.0, c.0) )
    }

    /// Dot product of two Vector3s: `self · other`.
    ///
    /// The result is returned as a `ScalarWrapper`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// let v1 = Vector3Wrapper([1.0_f64, 2.0, 3.0]);
    /// let v2 = Vector3Wrapper([0.1, 0.2, 0.3]);
    /// assert!((v1.dot(v2).unwrap() - 1.4).abs() < 1e-12);
    /// ```
    #[inline]
    pub fn dot(self, other: Vector3Wrapper<T>) -> ScalarWrapper<T> {
        ScalarWrapper( quat::dot(self.0, other.0) )
    }

    /// Cross product of two Vector3s: `self × other`.
    ///
    /// The product order is `self × other (!= other × self)`.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::{ScalarWrapper, Vector3Wrapper};
    /// let v1 = Vector3Wrapper([0.5_f64, -1.0, 0.8]);
    /// let v2 = v1 * ScalarWrapper(2.0);
    /// let result = v1.cross(v2).unwrap();
    /// assert!(result.iter().all(|value| value.abs() < 1e-12));
    /// ```
    #[inline]
    pub fn cross(self, other: Vector3Wrapper<T>) -> Self {
        Self( quat::cross(self.0, other.0) )
    }

    /// Calculate the L2 norm of the Vector3 and return it as a `ScalarWrapper`.
    ///
    /// Compared to `self.dot(self).unwrap().sqrt()`, this method is less likely
    /// to cause overflow and underflow.
    ///
    /// When the `norm-sqrt` feature is enabled, the default
    /// implementation is replaced with `self.dot(self).unwrap().sqrt()` internally.
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// let v = Vector3Wrapper([1.0_f64, 2.0, 3.0]);
    /// assert!((v.norm().unwrap() - 14.0_f64.sqrt()).abs() < 1e-12);
    ///
    /// let v = Vector3Wrapper([1e15_f32, 2e20, -3e15]);
    /// assert_eq!(v.dot(v).unwrap().sqrt(), f32::INFINITY);
    /// #[cfg(not(feature = "norm-sqrt"))]
    /// assert_eq!(v.norm().unwrap(), 2e20);
    /// ```
    #[inline]
    pub fn norm(self) -> ScalarWrapper<T> {
        ScalarWrapper( quat::norm(self.0) )
    }

    /// Normalize the Vector3.
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

    /// Calculate the inverse of the Pure Quaternion (Vector3).
    /// 
    /// # Examples
    /// 
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// let v = Vector3Wrapper::<f64>::new( [1.0, 2.0, 3.0] );
    /// 
    /// // Identity quaternion
    /// let id = (v * v.inv()).unwrap();  // = (v.inv() * v).unwrap()
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

    /// Exponential function of the Pure Quaternion (Vector3).
    ///
    /// # Examples
    ///
    /// ```
    /// # use quaternion_wrapper::Vector3Wrapper;
    /// let v = Vector3Wrapper([0.1_f64, 0.2, 0.3]);
    /// let diff = (v - v.exp().ln_versor()).unwrap();
    /// assert!(diff.iter().all(|value| value.abs() < 1e-12));
    /// ```
    #[inline]
    pub fn exp(self) -> QuaternionWrapper<T> {
        QuaternionWrapper( quat::exp(self.0) )
    }
}
