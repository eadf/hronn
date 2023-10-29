// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use super::*;
use vector_traits::glam::{DVec2, DVec3};

use crate::triangle_normal;
const EPSILON: f64 = 1e-10; // or some small value that's appropriate for your use case

fn assert_approx_eq<T: SillyApproxEq + Debug>(v1: T, v2: T, epsilon: f64) {
    assert!(v1.silly_approx_eq(&v2, epsilon), "{:?} != {:?}", v1, v2);
}

trait SillyApproxEq {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool;
}

impl SillyApproxEq for f64 {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool {
        (self - other).abs() <= epsilon
    }
}

impl SillyApproxEq for DVec2 {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool {
        (self.x - other.x).abs() <= epsilon && (self.y - other.y).abs() <= epsilon
    }
}

impl SillyApproxEq for DVec3 {
    fn silly_approx_eq(&self, other: &Self, epsilon: f64) -> bool {
        (self.x - other.x).abs() <= epsilon
            && (self.y - other.y).abs() <= epsilon
            && (self.z - other.z).abs() <= epsilon
    }
}

#[test]
fn test_plane_from_triangle() {
    let p0 = DVec3::new(50.0, 2.0, 7.0);
    let p1 = DVec3::new(10.0, 4.0, 1.0);
    let p2 = DVec3::new(0.9, 10.0, 10.0);

    let pft = PlaneFromTriangle::<DVec2>::new(p0, p1, p2);

    assert_approx_eq(pft.compute_z(p0.to_2d()), p0.z, EPSILON);
    assert_approx_eq(pft.compute_z(p1.to_2d()), p1.z, EPSILON);
    assert_approx_eq(pft.compute_z(p2.to_2d()), p2.z, EPSILON);

    let pft = PlaneFromTriangle::new_from_normal(triangle_normal(p0, p1, p2), p0);

    assert_approx_eq(pft.compute_z(p0.to_2d()), p0.z, EPSILON);
    assert_approx_eq(pft.compute_z(p1.to_2d()), p1.z, EPSILON);
    assert_approx_eq(pft.compute_z(p2.to_2d()), p2.z, EPSILON);

    let pft = PlaneFromTriangle::new_from_normal(triangle_normal(p0, p1, p2), p0);

    assert_approx_eq(pft.compute_z(p0.to_2d()), p0.z, EPSILON);
    assert_approx_eq(pft.compute_z(p1.to_2d()), p1.z, EPSILON);
    assert_approx_eq(pft.compute_z(p2.to_2d()), p2.z, EPSILON);
}

#[test]
fn test_rigid_transform_1() {
    use rand::prelude::*;
    //let origin = Vector2::default();
    let mut rng: StdRng = SeedableRng::seed_from_u64(42);
    for _ in 0..300 {
        let p0 = DVec3::new(
            rng.gen_range(0..50) as f64,
            rng.gen_range(0..50) as f64,
            rng.gen_range(0..50) as f64,
        );
        let p1 = DVec3::new(
            rng.gen_range(0..50) as f64,
            rng.gen_range(0..50) as f64,
            rng.gen_range(0..50) as f64,
        );
        let p2 = DVec2::new(rng.gen_range(0..50) as f64, rng.gen_range(0..50) as f64);
        if p0.to_2d() == p1.to_2d() {
            continue;
        }

        //println!("p0:{:?}", p0);
        //println!("p1:{:?}", p1);
        //println!("new origin=p2:{:?}\n", p2);

        // 2. Translate and rotate
        let transform = RigidTransform2D::translate_rotate_align_x(p2, p0.to_2d(), p1.to_2d());
        //println!("M:{:?}", transform);

        let transformed_p0 = transform.transform_point(p0);
        let transformed_p1 = transform.transform_point(p1);

        //println!("transformed_p0:{:?}", transformed_p0);
        //println!("transformed_p1:{:?}", transformed_p1);
        assert_approx_eq(transformed_p1.y, transformed_p0.y, EPSILON);
        assert_approx_eq(p0.z, transformed_p0.z, EPSILON);
        assert_approx_eq(p1.z, transformed_p1.z, EPSILON);

        // 3. Inverse the transformation
        let inverse_transform = transform.inverse().unwrap();
        let back_transformed_p0 = inverse_transform.transform_point(transformed_p0);
        let back_transformed_p1 = inverse_transform.transform_point(transformed_p1);
        //println!("back_transformed_p0:{:?}", back_transformed_p0);
        // 4. Test for equality
        assert_approx_eq(p0, back_transformed_p0, EPSILON);
        assert_approx_eq(p1, back_transformed_p1, EPSILON);
    }
}
