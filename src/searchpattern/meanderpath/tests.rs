// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use crate::{prelude::*, searchpattern::meanderpath::ConvexHullMeanderPath};
use linestring::linestring_2d::{convex_hull, Aabb2};

#[allow(unused_imports)]
use rayon::iter::IntoParallelIterator;
#[allow(unused_imports)]
use rayon::iter::ParallelIterator;
use vector_traits::{
    glam::{vec2, Vec2, Vec3},
    GenericVector2,
};

#[test]
fn test_meander_path_1() -> Result<(), HronnError> {
    let mut aabb = Aabb2::<Vec2>::new(vec2(0.0, 0.0));
    aabb.update_with_point(vec2(10.0, 10.0));
    let hull = aabb.convex_hull().unwrap();

    println!("hull:{:?}", hull);
    let zag_unit_vector = vec2(1.0, 0.0);
    let zig_unit_vector = vec2(0.0, 1.0);
    let start_point = vec2(0.0, 0.0);

    let path = ConvexHullMeanderPath::<Vec3, Vec3>::new(
        &hull,
        aabb,
        start_point,
        zig_unit_vector,
        zag_unit_vector,
        3.0,
    );

    let result: Result<Vec<_>, HronnError> = path
        .into_iter()?
        .map(|chunk_iter| {
            //println!("new chunk");
            let mut chunk_result = Vec::<Vec3>::new();
            for zag_iter in chunk_iter {
                let zag_iter = zag_iter?;
                /*println!(
                    "new zag iter: start:{} end:{} zags:{}",
                    zag_iter.start_pos, zag_iter.end_pos, zag_iter.current_zag
                );*/
                for sample in zag_iter {
                    assert!(
                        convex_hull::contains_point_inclusive(&hull, sample),
                        "{:?} is not inside {:?}",
                        sample,
                        hull
                    );
                    let sample = sample.to_3d(0.0);
                    println!("sample :{:?}", sample);
                    chunk_result.push(sample);
                }
            }
            Ok(chunk_result)
        })
        .collect();
    println!("result:{:?}", result?);
    Ok(())
}
