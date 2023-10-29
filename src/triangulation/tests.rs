// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use crate::{triangulation::triangulate_vertices, HronnError};
use linestring::linestring_2d::{Aabb2, LineString2};
use vector_traits::glam::{vec2, vec3, Vec2, Vec3};

#[test]
fn test_triangulate_meandering_vertices() -> Result<(), HronnError> {
    let vertices = vec![
        vec2(0.0, 0.0),
        vec2(1.0, 0.0),
        vec2(1.0, 1.0),
        vec2(0.0, 1.0),
    ];
    let aabb = Aabb2::<Vec2>::with_points(&vertices);
    let hull: LineString2<Vec2> = LineString2(aabb.convex_hull().unwrap());
    let samples = vec![
        vec3(0.3, 0.1, 0.2),
        vec3(0.3, 0.7, 0.2),
        vec3(0.1, 0.994, 0.2),
        vec3(0.4, 0.2, 0.6),
    ];
    let rv = triangulate_vertices::<Vec3, Vec3>(aabb, &hull.0, &samples)?;
    println!("Result: {:?}", rv);
    Ok(())
    //panic!()
}
