use quaternion_core as quat;
use quaternion_wrapper::*;

//const PI: f64 = std::f64::consts::PI;
const EPSILON: f64 = 1e-12;

#[test]
fn test_ops() {
    let q1: QuaternionWrapper<f64> = QuaternionWrapper::new( (1.0, [2.0, 3.0, 4.0]) );
    let q2: QuaternionWrapper<f64> = QuaternionWrapper::new( (0.1, [0.2, 0.3, 0.4]) );

    let q_result = (q1 + q2).unwrap();
    let q_check  = quat::add(q1.unwrap(), q2.unwrap());

    assert!( (q_check.0 - q_result.0).abs() < EPSILON );
    assert!( (q_check.1[0] - q_result.1[0]).abs() < EPSILON );
    assert!( (q_check.1[1] - q_result.1[1]).abs() < EPSILON );
    assert!( (q_check.1[2] - q_result.1[2]).abs() < EPSILON );
}

