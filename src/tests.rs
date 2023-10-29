// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use super::*;
use crate::{collision::meshanalyzer::SearchResult, prelude::MaximumTracker};

fn assert_approx_eq<T: ApproxEq + Debug>(v1: T, v2: T, epsilon: f64) {
    assert!(v1.approx_eq(&v2, epsilon), "{:?} != {:?}", v1, v2);
}

trait ApproxEq {
    fn approx_eq(&self, other: &Self, epsilon: f64) -> bool;
}

impl ApproxEq for f64 {
    fn approx_eq(&self, other: &Self, epsilon: f64) -> bool {
        (self - other).abs() <= epsilon
    }
}

#[test]
fn test_edge_collision_simple_0() {
    // this tests against an infinite edge, should give correct results when in range
    fn get_z(s_xy: DVec2, s_r: f64, p0: DVec3, p1: DVec3) -> f64 {
        let pol = closest_point_on_line_from_xy(s_xy, p0, p1);
        let dxy = (pol.to_2d() - s_xy).magnitude();

        let m_factor = m_factor_from_plane_unit_normal((p1 - p0).safe_normalize().unwrap());
        let z = pol.z + s_r * (1.0 - (dxy / s_r).powi(2)).sqrt() * m_factor;
        assert!(z.is_finite());
        z
    }

    fn get_z0(s_xy: DVec2, s_r: f64, p0: DVec3, p1: DVec3) -> f64 {
        let mut mt = MaximumTracker::<SearchResult<DVec2>>::default();
        probe::functions::ball_nose_to_edge_collision::<DVec2>(
            s_xy,
            s_r,
            0,
            p0,
            p1,
            probe::SkipEndpoint::NoSkip,
            &mut mt,
        );
        mt.get_max().map(|x| x.z_value).unwrap_or(f64::NEG_INFINITY)
    }

    fn get_z1(_s_xy: DVec2, _s_r: f64, _p0: DVec3, _p1: DVec3) -> f64 {
        /*probe::functions::z_coord_sphere_to_edge_1(s_xy, s_r, p0, p1, probe::SkipEndpoint::NoSkip)
        .unwrap_or(f64::NEG_INFINITY)*/
        0.0
    }

    fn get_z2(_s_xy: DVec2, _s_r: f64, _p0: DVec3, _p1: DVec3) -> f64 {
        /*crate::collision::probe::functions::z_coord_sphere_to_edge_2(s_xy, s_r, p0, p1)
        .unwrap_or(f64::NEG_INFINITY)*/
        0.0
    }

    fn get_z3(s_xy: DVec2, s_r: f64, p0: DVec3, p1: DVec3) -> f64 {
        let (x0, y0, z0) = (p0.x, p0.y, p0.z);
        let (x1, y1, z1) = (p1.x, p1.y, p1.z);
        let (s_x, s_y, r) = (s_xy.x, s_xy.y, s_r);
        let s_x = s_x;
        let s_y = s_y;
        //let z = (s_x *x0*z0 - s_x *x0*z1 - s_x *x1*z0 + s_x *x1*z1 + s_y *y0*z0 - s_y *y0*z1 - s_y *y1*z0 + s_y *y1*z1 + x0.powi(2)*z1 - x0*x1*z0 - x0*x1*z1 + x1.powi(2)*z0 + y0.powi(2)*z1 - y0*y1*z0 - y0*y1*z1 + y1.powi(2)*z0 - f64::sqrt(-(x0.powi(2) - 2.0*x0*x1 + x1.powi(2) + y0.powi(2) - 2.0*y0*y1 + y1.powi(2) + z0.powi(2) - 2.0*z0*z1 + z1.powi(2))*(s_x.powi(2)*y0.powi(2) - 2.0* s_x.powi(2)*y0*y1 + s_x.powi(2)*y1.powi(2) - 2.0* s_x * s_y *x0*y0 + 2.0* s_x * s_y *x0*y1 + 2.0* s_x * s_y *x1*y0 - 2.0* s_x * s_y *x1*y1 + 2.0* s_x *x0*y0*y1 - 2.0* s_x *x0*y1.powi(2) - 2.0* s_x *x1*y0.powi(2) + 2.0* s_x *x1*y0*y1 + s_y.powi(2)*x0.powi(2) - 2.0* s_y.powi(2)*x0*x1 + s_y.powi(2)*x1.powi(2) - 2.0* s_y *x0.powi(2)*y1 + 2.0* s_y *x0*x1*y0 + 2.0* s_y *x0*x1*y1 - 2.0* s_y *x1.powi(2)*y0 - r.powi(2)*x0.powi(2) + 2.0*r.powi(2)*x0*x1 - r.powi(2)*x1.powi(2) - r.powi(2)*y0.powi(2) + 2.0*r.powi(2)*y0*y1 - r.powi(2)*y1.powi(2) + x0.powi(2)*y1.powi(2) - 2.0*x0*x1*y0*y1 + x1.powi(2)*y0.powi(2))))/(x0.powi(2) - 2.0*x0*x1 + x1.powi(2) + y0.powi(2) - 2.0*y0*y1 + y1.powi(2));
        let z = (s_x * x0 * z0 - s_x * x0 * z1 - s_x * x1 * z0 + s_x * x1 * z1 + s_y * y0 * z0
            - s_y * y0 * z1
            - s_y * y1 * z0
            + s_y * y1 * z1
            + x0.powi(2) * z1
            - x0 * x1 * z0
            - x0 * x1 * z1
            + x1.powi(2) * z0
            + y0.powi(2) * z1
            - y0 * y1 * z0
            - y0 * y1 * z1
            + y1.powi(2) * z0
            + f64::sqrt(
                -(x0.powi(2) - 2.0 * x0 * x1 + x1.powi(2) + y0.powi(2) - 2.0 * y0 * y1
                    + y1.powi(2)
                    + z0.powi(2)
                    - 2.0 * z0 * z1
                    + z1.powi(2))
                    * (s_x.powi(2) * y0.powi(2) - 2.0 * s_x.powi(2) * y0 * y1
                        + s_x.powi(2) * y1.powi(2)
                        - 2.0 * s_x * s_y * x0 * y0
                        + 2.0 * s_x * s_y * x0 * y1
                        + 2.0 * s_x * s_y * x1 * y0
                        - 2.0 * s_x * s_y * x1 * y1
                        + 2.0 * s_x * x0 * y0 * y1
                        - 2.0 * s_x * x0 * y1.powi(2)
                        - 2.0 * s_x * x1 * y0.powi(2)
                        + 2.0 * s_x * x1 * y0 * y1
                        + s_y.powi(2) * x0.powi(2)
                        - 2.0 * s_y.powi(2) * x0 * x1
                        + s_y.powi(2) * x1.powi(2)
                        - 2.0 * s_y * x0.powi(2) * y1
                        + 2.0 * s_y * x0 * x1 * y0
                        + 2.0 * s_y * x0 * x1 * y1
                        - 2.0 * s_y * x1.powi(2) * y0
                        - r.powi(2) * x0.powi(2)
                        + 2.0 * r.powi(2) * x0 * x1
                        - r.powi(2) * x1.powi(2)
                        - r.powi(2) * y0.powi(2)
                        + 2.0 * r.powi(2) * y0 * y1
                        - r.powi(2) * y1.powi(2)
                        + x0.powi(2) * y1.powi(2)
                        - 2.0 * x0 * x1 * y0 * y1
                        + x1.powi(2) * y0.powi(2)),
            ))
            / (x0.powi(2) - 2.0 * x0 * x1 + x1.powi(2) + y0.powi(2) - 2.0 * y0 * y1 + y1.powi(2));
        assert!(z.is_finite());
        z
    }
    fn get_z4(s_xy: DVec2, s_r: f64, p0: DVec3, p1: DVec3) -> f64 {
        let (x0, y0, z0) = (p0.x, p0.y, p0.z);
        let (x1, y1, z1) = (p1.x, p1.y, p1.z);
        let (s_x, s_y, r) = (s_xy.x, s_xy.y, s_r);
        let s_x = s_x;
        let s_y = s_y;

        let z = -r
            * (z0
                + (-z0 + z1)
                    * (-s_x * x0 + s_x * x1 - s_y * y0 + s_y * y1 + x0.powi(2) - x0 * x1
                        + y0.powi(2)
                        - y0 * y1)
                    / (x0.powi(2) - 2.0 * x0 * x1 + x1.powi(2) + y0.powi(2) - 2.0 * y0 * y1
                        + y1.powi(2)))
            * ((-s_x
                + x0
                + (-x0 + x1)
                    * (-s_x * x0 + s_x * x1 - s_y * y0 + s_y * y1 + x0.powi(2) - x0 * x1
                        + y0.powi(2)
                        - y0 * y1)
                    / (x0.powi(2) - 2.0 * x0 * x1 + x1.powi(2) + y0.powi(2) - 2.0 * y0 * y1
                        + y1.powi(2)))
            .powi(2)
                + (-s_y
                    + y0
                    + (-y0 + y1)
                        * (-s_x * x0 + s_x * x1 - s_y * y0 + s_y * y1 + x0.powi(2) - x0 * x1
                            + y0.powi(2)
                            - y0 * y1)
                        / (x0.powi(2) - 2.0 * x0 * x1 + x1.powi(2) + y0.powi(2) - 2.0 * y0 * y1
                            + y1.powi(2)))
                .powi(2)
                + (z0
                    + (-z0 + z1)
                        * (-s_x * x0 + s_x * x1 - s_y * y0 + s_y * y1 + x0.powi(2) - x0 * x1
                            + y0.powi(2)
                            - y0 * y1)
                        / (x0.powi(2) - 2.0 * x0 * x1 + x1.powi(2) + y0.powi(2) - 2.0 * y0 * y1
                            + y1.powi(2)))
                .powi(2))
            + z0
            + (-z0 + z1)
                * (-s_x * x0 + s_x * x1 - s_y * y0 + s_y * y1 + x0.powi(2) - x0 * x1 + y0.powi(2)
                    - y0 * y1)
                / (x0.powi(2) - 2.0 * x0 * x1 + x1.powi(2) + y0.powi(2) - 2.0 * y0 * y1
                    + y1.powi(2));

        /*let t = (-s_x * x0 + s_x * x1 - s_y * y0 + s_y * y1 + x0.powi(2) - x0 * x1 + y0.powi(2)
        - y0 * y1)
        / (x0.powi(2) - 2.0 * x0 * x1 + x1.powi(2) + y0.powi(2) - 2.0 * y0 * y1 + y1.powi(2));
        */

        //println!("t:{}", t);
        assert!(z.is_finite());
        z
    }

    let p0 = DVec3::new(2.0, 0.0, 0.0);
    let p1 = DVec3::new(18.0, 0.0, 5.0);
    //let p2_virtual = Vector3::new(2.0, 10.0, 0.0);
    let s_r = 0.9;

    //println!("m:{}", calculate_m(p1 - p0));

    for i in 0..21 {
        let x = i as f64;
        let s_xy = DVec2::new(x, 0.8999999999);
        println!(
            "{:2.1} z={:3.4},z0={:3.4} z1={:.4} z2={:.4} z3={:.4} z4={:.4}",
            s_xy,
            get_z(s_xy, s_r, p0, p1),
            get_z0(s_xy, s_r, p0, p1),
            get_z1(s_xy, s_r, p0, p1),
            get_z2(s_xy, s_r, p0, p1),
            get_z3(s_xy, s_r, p0, p1),
            get_z4(s_xy, s_r, p0, p1)
        );
    }

    //panic!();
}

/*
#[test]
fn test_edge_collision_horizontal() {
    // test this obj file:
    // v 1.460996 4.977283 2.261945
    // v 2.332969 3.777106 2.261950
    // v 1.720666 4.249082 1.000000
    // f 1 3 2
    let start_vertex = Vector3::new(1.460996, 4.977283, 2.261945);
    let end_vertex = Vector3::new(2.332969, 3.777106, 2.261950);
    let probe_radius = 0.5;

    let center = start_vertex.to_2d();
    let collision = probe::functions::ball_nose_to_edge_collision(
        center,
        probe_radius,
        start_vertex,
        end_vertex,
        probe::SkipEndpoint::NoSkip,
    );
    assert!(collision.is_some());
    assert_approx_eq(collision.unwrap(), start_vertex.z + probe_radius, EPSILON);
    let collision = probe::functions::z_coord_sphere_to_edge_1(
        center,
        probe_radius,
        start_vertex,
        end_vertex,
        probe::SkipEndpoint::NoSkip,
    );
    assert!(collision.is_some());
    assert_approx_eq(collision.unwrap(), start_vertex.z + probe_radius, EPSILON);

    let center = end_vertex.to_2d();
    let collision = probe::functions::ball_nose_to_edge_collision(
        center,
        probe_radius,
        start_vertex,
        end_vertex,
        probe::SkipEndpoint::NoSkip,
    );
    assert!(collision.is_some());
    assert_approx_eq(collision.unwrap(), end_vertex.z + probe_radius, EPSILON);
    let collision = crate::collision::probe::functions::z_coord_sphere_to_edge_1(
        center,
        probe_radius,
        start_vertex,
        end_vertex,
        probe::SkipEndpoint::NoSkip,
    );
    assert!(collision.is_some());
    assert_approx_eq(collision.unwrap(), end_vertex.z + probe_radius, EPSILON);

    let middle = start_vertex + 0.5 * (end_vertex - start_vertex);
    let center = middle.to_2d();
    let collision = probe::functions::ball_nose_to_edge_collision(
        center,
        probe_radius,
        start_vertex,
        end_vertex,
        probe::SkipEndpoint::NoSkip,
    );
    assert!(collision.is_some());
    assert_approx_eq(collision.unwrap(), middle.z + probe_radius, EPSILON);
    let middle = start_vertex + 0.5 * (end_vertex - start_vertex);
    let center = middle.to_2d();
    let collision = probe::functions::z_coord_sphere_to_edge_1(
        center,
        probe_radius,
        start_vertex,
        end_vertex,
        probe::SkipEndpoint::NoSkip,
    );
    assert!(collision.is_some());
    assert_approx_eq(collision.unwrap(), middle.z + probe_radius, EPSILON);
}
*/
/*
#[test]
fn test_edge_collision_simple() {
    let start_vertex = Vector3 {
        x: -2.0,
        y: 0.0,
        z: 0.0,
    };
    let end_vertex = Vector3 {
        x: 2.0,
        y: 0.0,
        z: 0.0,
    };

    let center = Vector2 { x: 1.0, y: 0.0 };
    let radius = 1.0;

    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex);
    assert!(collision.is_some());
    let collision = collision.unwrap();
    assert_approx_eq(collision, 1.0, EPSILON);

    let center = Vector2 { x: -1.0, y: 0.0 };

    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex);
    assert!(collision.is_some());
    let collision = collision.unwrap();
    assert_approx_eq(collision, 1.0, EPSILON);

    let circle = Vector2 { x: -2.0, y: 0.0 };
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(circle, radius, end_vertex, start_vertex);
    assert!(collision.is_some());
    let collision = collision.unwrap();
    assert_approx_eq(collision, 1.0, EPSILON);

    let center = Vector2 { x: 2.0, y: 0.0 };
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex);
    assert!(collision.is_some());
    let collision = collision.unwrap();
    assert_approx_eq(collision, 1.0, EPSILON);

    let center = Vector2 { x: -2.0, y: -0.5 };
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, end_vertex, start_vertex);
    assert!(collision.is_some());
    let collision = collision.unwrap();
    println!("collision at {:?}", collision);
    assert_approx_eq(collision, 0.8660254037844386, EPSILON);
}*/

/*
#[test]
fn test_edge_collision_point_1() {
    let center = Vector2 { x: 5.0, y: 0.0 };
    let radius = 1.0;

    let start_vertex = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let end_vertex = Vector3 {
        x: 10.0,
        y: 0.0,
        z: 10.0,
    };

    // Check if the function returns the correct collision point
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex);
    assert!(collision.is_some());
    let collision = collision.unwrap();
    assert!(collision.is_normal());
    // Checking the equality with a tolerance for floating point precision issues
    assert_approx_eq(collision, 6.414213562373095, EPSILON);

    // Check if the function returns None for a sphere that doesn't touch the edge
    let center = Vector2 {
        x: 2.0,
        y: 2.0000001,
    };
    assert!(
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex)
            .is_none()
    );

    let center = Vector2 { x: 50.0, y: 50.0 };
    // Check if the function returns the correct collision point
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex);
    assert!(
        collision.is_none(),
        "{:?} was suppose to be None",
        collision
    );

    let center = Vector2 { x: -1.0, y: -1.0 };
    // Check if the function returns the correct collision point
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex);
    assert!(
        collision.is_none(),
        "{:?} was suppose to be None",
        collision
    );

    let center = Vector2 { x: 0.0, y: -11.0 };
    // Check if the function returns the correct collision point
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex);
    assert!(
        collision.is_none(),
        "{:?} was suppose to be None",
        collision
    );
}
*/
/*
#[test]
fn test_edge_collision_point_2() {
    let center = Vector2 {
        x: 9.99999999,
        y: 0.0,
    };
    let radius = 1.0;

    let start_vertex = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let end_vertex = Vector3 {
        x: 10.0,
        y: 0.0,
        z: 10.0,
    };

    // Check if the function returns the correct collision point
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, end_vertex, start_vertex);
    assert!(collision.is_some());
    let collision = collision.unwrap();
    assert!(collision.is_normal());
    // Checking the equality with a tolerance for floating point precision issues
    assert!(
        (collision - 10.999999999999999999).abs() < 1e-9,
        "{} was not close to 11.0",
        collision
    );
}

#[test]
fn test_edge_collision_point_3() {
    let radius = f64::from(3.0);

    let start_vertex = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let end_vertex = Vector3 {
        x: 10.0,
        y: 0.0,
        z: 0.0,
    };

    let center = Vector2 { x: 0.0, y: 0.0 };

    // Check if the function returns the correct collision point
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, end_vertex, start_vertex)
            .unwrap();
    assert_approx_eq(collision, radius, EPSILON);

    let center = Vector2 { x: 5.0, y: 0.0 };
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex)
            .unwrap();
    assert_approx_eq(collision, radius, EPSILON);

    let center = Vector2 { x: 10.0, y: 0.0 };
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex)
            .unwrap();
    assert_approx_eq(collision, radius, EPSILON);
}

#[test]
fn test_edge_collision_point_4() {
    let radius = f64::from(3.0);

    let start_vertex = Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let end_vertex = Vector3 {
        x: 10.0,
        y: 0.0,
        z: 1.0,
    };

    let center = Vector2 { x: 0.0, y: 0.0 };
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, end_vertex, start_vertex)
            .unwrap();
    assert_approx_eq(collision, radius + 0.014962686336267, EPSILON);

    let center = Vector2 { x: 5.0, y: 0.0 };
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, end_vertex)
            .unwrap();
    assert_approx_eq(
        collision,
        0.014962686336267 + radius + end_vertex.z / 2.0,
        EPSILON,
    );

    let center = Vector2 { x: 10.0, y: 0.0 };
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, end_vertex, start_vertex)
            .unwrap();
    assert_approx_eq(collision, radius + end_vertex.z, EPSILON);
}

#[test]
fn test_edge_collision_point_5() {
    let radius = f64::from(3.0);

    let start_vertex = Vector3::new(0.0, 0.0, 0.0);
    let end_vertex = Vector3::new(5.0, 0.0, 0.0);
    let top_vertex = Vector3::new(0.0, 5.0, 5.0);

    let center = top_vertex.to_2d();
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, top_vertex, end_vertex).unwrap();
    assert_approx_eq(collision, top_vertex.z + radius, EPSILON);

    let center = top_vertex.to_2d() + Vector2::new(0.0, radius);

    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, top_vertex, end_vertex).unwrap();
    assert_approx_eq(collision, top_vertex.z, EPSILON);

    let center = Vector2::new(-radius + 0.000000000001, 5.0);
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, top_vertex, start_vertex)
            .unwrap();
    assert_approx_eq(collision, 5.0, 0.00001);

    let center = Vector2::new(-radius + 0.000000000001, 2.5);
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, top_vertex)
            .unwrap();
    assert_approx_eq(collision, 2.5, 0.00001);

    let center = Vector2::new(-radius + 0.000000000001, 0.0);
    let collision =
        probe::functions::z_coord_sphere_to_edge_2(center, radius, start_vertex, top_vertex)
            .unwrap();
    assert_approx_eq(collision, 0.0, 0.00001);
}*/

/*
#[test]
fn test_face_collision_point_1() {
    let vertices = vec![
        DVec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        DVec3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        DVec3 {
            x: 0.0,
            y: 1.0,
            z: 0.0,
        },
        DVec3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        },
    ];

    let triangle = [0, 1, 2];

    // Case 1: Circle above triangle, expected to collide at z = -0.5
    let circle = Circle {
        center: DVec2 { x: 0.5, y: 0.5 },
        radius: 0.5,
    };
    let result = face_collision_point(&circle, &vertices, &triangle, -f64::INFINITY);
    assert!(result.is_some());
    assert!(
        (result.unwrap().z - 0.5).abs() < 1e-10,
        "{:?} was not 0.5",
        result
    );

    // Case 2: Circle too far away from triangle, expect no collision
    let circle = Circle {
        center: DVec2 { x: 2.0, y: 2.0 },
        radius: 0.5,
    };
    let result = face_collision_point(&circle, &vertices, &triangle, -f64::INFINITY);
    assert!(result.is_none());
}*/

#[test]
fn test_point_inside_triangle_1() {
    let p0 = DVec2::new(0.0, 0.0);
    let p1 = DVec2::new(5.0, 0.0);
    let p2 = DVec2::new(0.0, 5.0);
    let q = DVec2::new(2.5, 2.5);

    assert!(is_inside_2d_triangle(q, p0, p1, p2,));
}

#[test]
fn test_point_inside_triangle_2() {
    let p0 = DVec2::new(0.7071067811865478, -0.0);
    let p1 = DVec2::new(5.0, 0.0);
    let p2 = DVec2::new(5.0, 4.292893218813452);

    let q = DVec2::new(2.0, 1.0);

    assert!(
        is_inside_2d_triangle(q, p0, p1, p2),
        "[{:?},{:?},{:?}]",
        p0,
        p1,
        p2,
    );
}

#[test]
fn test_interpolate_z_line() {
    let q = DVec2::new(5.0, 0.0);
    let p0 = DVec3::new(0.0, 0.0, 0.0);
    let p1 = DVec3::new(10.0, 0.0, 10.0);

    let z = interpolate_z_line(q, p0, p1);
    assert_approx_eq(z, 5.0, EPSILON);
}

#[test]
fn test_point_outside_triangle_1() {
    let p0 = DVec2::new(0.0, 0.0);
    let p1 = DVec2::new(5.0, 0.0);
    let p2 = DVec2::new(0.0, 5.0);
    let q = DVec2::new(-2.5, 2.5);

    assert!(!is_inside_2d_triangle(q, p0, p1, p2));
}

#[test]
fn test_point_outside_triangle_2() {
    let p0 = DVec2::new(0.0, 0.0);
    let p1 = DVec2::new(0.0, 10.0);
    let p2 = DVec2::new(10.0, 0.0);
    let q = DVec2::new(10.0, -0.8944271909999159);

    assert!(!is_inside_2d_triangle(q, p0, p1, p2));
}

#[test]
fn test_point_on_edge_of_triangle() {
    let p0 = DVec2::new(0.0, 0.0);
    let p1 = DVec2::new(5.0, 0.0);
    let p2 = DVec2::new(0.0, 5.0);
    let q = DVec2::new(0.0, 2.5); // On the edge between p0 and p2

    assert!(is_inside_2d_triangle(q, p0, p1, p2));
}

#[test]
fn test_point_on_vertex_of_triangle() {
    let p0 = DVec2::new(0.0, 0.0);
    let p1 = DVec2::new(5.0, 0.0);
    let p2 = DVec2::new(0.0, 5.0);
    let q = p1.clone(); // Directly on p1

    assert!(is_inside_2d_triangle(q, p0, p1, p2));
}

#[test]
fn test_squared_distance_line_point() {
    let p = DVec2::new(2.0, 3.0);
    let a = DVec2::new(0.0, 0.0);
    let b = DVec2::new(4.0, 0.0);

    assert_approx_eq(distance_sq_point_to_segment(p, a, b), 9.0, EPSILON);

    let p = DVec2::new(0.0, 9.0);
    let a = DVec2::new(0.0, 0.0);
    let b = DVec2::new(10.0, 0.0);

    assert_approx_eq(distance_sq_point_to_segment(p, a, b), 81.0, EPSILON);

    let p = DVec2::new(3.0, 0.0);
    let a = DVec2::new(0.0, 0.0);
    let b = DVec2::new(10.0, 0.0);

    assert_approx_eq(distance_sq_point_to_segment(p, a, b), 0.0, EPSILON);

    let p = DVec2::new(1.0, 1.0);
    let a = DVec2::new(0.0, 0.0);
    let b = DVec2::new(0.0, 2.0);

    let dist_sq = distance_sq_point_to_line(p, a, b);
    assert_approx_eq(dist_sq, 1.0, EPSILON);
}

#[test]
fn reconstruct_from_unordered_edges_1() -> Result<(), CollisionError> {
    let indices = vec![0_usize, 1];
    let result = reconstruct_from_unordered_edges(&indices)?;
    println!("result1:{:?}", result);
    assert_eq!(result, vec![0_usize, 1]);

    let indices = vec![0_usize, 1, 1, 2, 2, 3, 3, 4];
    let result = reconstruct_from_unordered_edges(&indices)?;
    println!("result1:{:?}", result);
    assert_eq!(result, vec![0_usize, 1, 2, 3, 4]);

    let indices = vec![0_usize, 1, 1, 2, 2, 3, 3, 4, 4, 0];
    let result = reconstruct_from_unordered_edges(&indices)?;
    println!("result1:{:?}", result);
    assert_eq!(result, vec![0_usize, 1, 2, 3, 4, 0]);
    Ok(())
}
