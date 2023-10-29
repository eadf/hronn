// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.
#[cfg(test)]
mod tests;
use crate::{prelude::RigidTransform2D, HronnError};
use linestring::linestring_2d::{Aabb2, LineString2};
use rayon::prelude::IntoParallelIterator;
use smallvec::alloc;
use std::marker::PhantomData;
use std::ops::Add;
use std::ops::Neg;
use std::ops::Sub;
use vector_traits::{
    num_traits::{real::Real, AsPrimitive},
    Approx, GenericScalar, GenericVector2, GenericVector3, HasXY, HasXYZ,
};

#[derive(Debug)]
#[allow(dead_code)]
pub struct ConvexHullMeanderPath<'a, T: GenericVector3, MESH: HasXYZ> {
    hull: &'a LineString2<T::Vector2>,
    aabb: Aabb2<T::Vector2>,
    zig_vector: T::Vector2,
    number_of_zigs: u32,
    zag_vector: T::Vector2,
    zag_unit_vector: T::Vector2,
    current_pos: T::Vector2,
    last_sample_point: Option<T::Vector2>,
    step: T::Scalar,
    #[doc(hidden)]
    _pd: PhantomData<MESH>,
}

impl<'a, T: GenericVector3, MESH: HasXYZ> ConvexHullMeanderPath<'a, T, MESH>
where
    T::Scalar: AsPrimitive<MESH::Scalar> + AsPrimitive<f32> + AsPrimitive<u32>,
    MESH: Approx,
    u32: AsPrimitive<MESH::Scalar>,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        hull: &'a LineString2<T::Vector2>,
        aabb: Aabb2<T::Vector2>,
        start_point: T::Vector2,
        zig_unit_vector: T::Vector2,
        zag_unit_vector: T::Vector2,
        step: T::Scalar,
    ) -> Self {
        ConvexHullMeanderPath {
            hull,
            aabb,
            zig_vector: zig_unit_vector * step,
            number_of_zigs: 0,
            zag_vector: zag_unit_vector * step,
            zag_unit_vector,
            current_pos: start_point,
            last_sample_point: None,
            step,
            _pd: PhantomData,
        }
    }

    // An attempt at dividing up a length in manageable chunks
    // It will try to create chunks large enough to fit at least two `step_size` in each.
    pub fn calculate_chunk_limits(
        mut max_side: T::Scalar,
        mut step_size: T::Scalar,
        max_chunks: T::Scalar,
    ) -> Vec<(T::Scalar, T::Scalar, u32)> {
        max_side = max_side.abs();
        step_size = step_size.abs();

        let total_steps: T::Scalar = (max_side / step_size).ceil();

        let mut chunks = (total_steps / 2.0.into()).ceil();
        if chunks > max_chunks {
            chunks = max_chunks;
        }

        let steps_per_chunk = ((total_steps / chunks).floor()) * T::Scalar::TWO;
        let mut start = T::Scalar::ZERO;

        let mut boundaries: Vec<(T::Scalar, T::Scalar, u32)> = Vec::new();
        let chunks: u32 = chunks.as_();

        for _ in 0..chunks {
            let end = (start + steps_per_chunk * step_size).min(max_side);
            // Skip redundant chunks
            if (end - start).abs() < step_size * T::Scalar::TWO {
                break;
            }
            boundaries.push((start, end, steps_per_chunk.as_()));
            start = end;
        }

        // Handle any leftover section
        if start < max_side {
            let t: T::Scalar = ((max_side - start) / step_size).floor();
            boundaries.push((start, max_side, t.as_()));
        }

        boundaries
    }

    fn generate_chunks(&self) -> Result<Vec<ChunkIter<'a, T, MESH>>, HronnError> {
        let zag_unit_vector = T::Vector2::new_2d(T::Scalar::ONE, T::Scalar::ZERO);
        let zig_unit_vector = T::Vector2::new_2d(T::Scalar::ZERO, T::Scalar::ONE);
        let zig_vector = zig_unit_vector * self.step;
        let zag_vector = zag_unit_vector * self.step;

        // create a rotation matrix that aligns the zag vector with the X axis
        let transform = RigidTransform2D::<T>::translate_rotate_align_x(
            T::Vector2::new_2d(T::Scalar::ZERO, T::Scalar::ZERO),
            T::Vector2::new_2d(T::Scalar::ZERO, T::Scalar::ZERO),
            zag_unit_vector,
        );

        // create a zag axis AABB
        let zap_bb = {
            let mut zap_bb = Aabb2::default();
            zap_bb.update_with_point(
                transform
                    .transform_point(self.aabb.low().unwrap().to_3d(T::Scalar::ZERO))
                    .to_2d(),
            );
            zap_bb.update_with_point(
                transform
                    .transform_point(self.aabb.high().unwrap().to_3d(T::Scalar::ZERO))
                    .to_2d(),
            );
            zap_bb
        };

        let zip_bb_side_length = zap_bb.high().unwrap().y() - zap_bb.low().unwrap().y();
        //println!("zip_bb_side_length:{:.3}", zip_bb_side_length);
        // define the chunk ranges in zig zag space
        let chunks = ConvexHullMeanderPath::<T, MESH>::calculate_chunk_limits(
            zip_bb_side_length,
            self.step,
            8.0.into(),
        );
        if chunks.is_empty() {
            println!(
                "No chunks found. zip_bb_side_length:{:?} aabb:{:?}",
                zip_bb_side_length, self.aabb
            );
        }
        /*for (i, (start, end, steps)) in chunks.iter().enumerate() {
            println!(
                "Chunk {}: Start = {}, End = {} steps = {}",
                i + 1,
                start,
                end,
                steps
            );
        }*/

        let transform_inverse = transform.inverse().unwrap();
        let zap_bb_low = zap_bb.low().unwrap();
        //println!("zap_bb_low:{:.3}", zap_bb_low);
        let safe_distance = self.aabb.low().unwrap().distance(self.aabb.high().unwrap());
        //println!("safe distance:{:.3}", safe_distance);

        // define the chunk ranges in "normal" space
        let chunks: Vec<(T::Vector2, T::Vector2, u32)> = {
            // move the start point back a bit
            let safety_margin = zag_unit_vector * safe_distance;
            //println!("zag_unit_vector:{:.3}", zag_unit_vector);
            //println!("zig_unit_vector:{:.3}", zig_unit_vector);
            //println!("zag_vector:{:.3}", zag_vector);
            //println!("zig_vector:{:.3}", zig_vector);
            let zig_unit_vector_t = transform
                .transform_point(zig_unit_vector.to_3d(T::Scalar::ZERO))
                .to_2d();
            //println!("zig_unit_vector_t:{:.3}", zig_unit_vector_t);
            //println!("safety_margin:{:.3}", safety_margin);
            chunks
                .into_iter()
                .map(|(chunk_start, chunk_end, chunk_steps)| {
                    /*println!(
                        "chunk_start:{:.3}, chunk_end:{:.3} chunk_steps:{}",
                        chunk_start, chunk_end, chunk_steps
                    );*/
                    let start_p = T::Vector2::new_2d(
                        zap_bb_low.x(),
                        zap_bb_low.y() + zig_unit_vector_t.y() * chunk_start,
                    );
                    let start_p = (transform_inverse
                        .transform_point(start_p.to_3d(T::Scalar::ZERO)))
                    .to_2d()
                    .sub(safety_margin);
                    let end_p = T::Vector2::new_2d(
                        zap_bb_low.x(),
                        zap_bb_low.y() + zig_unit_vector_t.y() * chunk_end,
                    );
                    //println!("start_p:{:.3}, end_p:{:.3}", start_p, end_p);
                    let end_p = transform_inverse
                        .transform_point(end_p.to_3d(T::Scalar::ZERO))
                        .to_2d();
                    (start_p, end_p, chunk_steps)
                })
                .collect()
        };
        if chunks.is_empty() {
            return Err(HronnError::InternalError(
                "Could not generate the expected chunks.".to_string(),
            ));
        }
        /*println!();
        for (i, (start, end, steps)) in chunks.iter().enumerate() {
            println!(
                "Chunk {}: Start = {:?},  end = {:?} steps = {}",
                i, start, end, steps
            );
        }
        {
            let min = self.aabb.get_low().unwrap();
            let max = self.aabb.get_high().unwrap();
            println!("aabb: min:{:?} max:{:?}", min, max);
            println!();
        }*/

        let zig_vector = {
            let start_point = chunks.first().unwrap().0;
            let dot = (start_point.sub(self.aabb.center().unwrap())).dot(zig_vector);
            if dot.is_sign_negative() {
                //println!("dot < 0 -> zig = {:?}", zig_vector);
                zig_vector
            } else {
                //println!("dot>= 0 -> zig = {:?}", zig_vector.neg());
                zig_vector.neg()
            }
        };

        Ok(chunks
            .into_iter()
            .enumerate()
            .map(|(chunk_id, (start_p, end_p, chunk_steps))| {
                ChunkIter::<T, MESH>::new(
                    self.hull,
                    chunk_id,
                    start_p,
                    end_p,
                    self.zag_unit_vector,
                    zag_vector,
                    chunk_steps,
                    zig_vector,
                    safe_distance,
                    self.step,
                )
            })
            .collect())
    }

    #[allow(dead_code)]
    pub fn into_iter(self) -> Result<alloc::vec::IntoIter<ChunkIter<'a, T, MESH>>, HronnError> {
        Ok(self.generate_chunks()?.into_iter())
    }

    /// this is a bit non-standard for a .par_iter() but it makes error handling easier
    #[allow(dead_code)]
    pub fn into_par_iter(self) -> Result<rayon::vec::IntoIter<ChunkIter<'a, T, MESH>>, HronnError> {
        Ok(self.generate_chunks()?.into_par_iter())
    }
}

#[derive(Debug)]
pub struct ChunkIter<'a, T: GenericVector3, MESH: HasXYZ> {
    hull: &'a LineString2<T::Vector2>,
    chunk_id: usize,
    start_point: T::Vector2,
    #[allow(dead_code)]
    end_point: T::Vector2,
    current_pos: T::Vector2,

    zag_vector: T::Vector2,
    zag_ray: T::Vector2,
    zag_entry: Option<T::Vector2>,
    zig_vector: T::Vector2,
    current_zigs: u32,
    n_zig_limit: u32,
    safe_distance: T::Scalar,
    zag_magnitude: T::Scalar,
    #[doc(hidden)]
    _pdm: PhantomData<MESH>,
}

/// Does zig iterations over one chunk
impl<'a, T: GenericVector3, MESH: HasXYZ> ChunkIter<'a, T, MESH> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        hull: &'a LineString2<T::Vector2>,
        chunk_id: usize,
        start_point: T::Vector2,
        end_point: T::Vector2,
        mut zag_ray: T::Vector2,
        mut zag_vector: T::Vector2,
        n_zig_limit: u32,
        zig_vector: T::Vector2,
        safe_distance: T::Scalar,
        step: T::Scalar,
    ) -> Self {
        let current_pos = start_point;
        let mut zag_entry = hull.closest_ray_intersection(zag_ray, current_pos);

        if zag_entry.is_none() {
            zag_vector = zag_vector.neg();
            zag_ray = zag_ray.neg();
            println!("zag was the wrong way");
            zag_entry = hull.closest_ray_intersection(zag_ray, current_pos);
        }
        //println!("ChunkIter::new() start:{:?}, end:{:?},", start_point, end_point);
        Self {
            hull,
            chunk_id,
            current_pos,
            zag_vector,
            start_point,
            end_point,
            zag_ray,
            zag_entry,
            current_zigs: 0,
            n_zig_limit,
            zig_vector,
            safe_distance,
            zag_magnitude: step,
            _pdm: PhantomData,
        }
    }
}

impl<'a, T: GenericVector3, MESH: HasXYZ> Iterator for ChunkIter<'a, T, MESH> {
    type Item = Result<ZagIterator<T>, HronnError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current_zigs >= self.n_zig_limit {
            /*println!(
                "ChId:{} zig out at {}>={} pos:{}",
                self.chunk_id, self.current_zigs, self.n_zig_limit, self.current_pos
            );*/
            return None;
        }

        if let Some(entry) = self.zag_entry {
            self.current_pos = entry;
            let start_pos = entry;

            // Zagging inside the hull, determine the exit point by ray casting back from a point
            // far ahead of current position
            let look_ahead_point = self.current_pos.add(self.zag_ray * self.safe_distance);
            return if let Some(zag_exit) = self
                .hull
                .closest_ray_intersection(self.zag_ray.neg(), look_ahead_point)
            {
                let distance_to_exit = (zag_exit.sub(self.current_pos)).magnitude();
                /*assert!(
                    !ulps_eq!(distance_to_exit, 0.0),
                    "distance_to_exit:{}",
                    distance_to_exit
                );*/
                /*println!(
                    "distance_to_exit:{distance_to_exit}, zag_magnitude:{}",
                    self.zag_magnitude
                );*/
                let n_zag_limit: u32 = (distance_to_exit / self.zag_magnitude).ceil().as_();
                //println!("n_zag_limit:{n_zag_limit}, distance_to_exit:{distance_to_exit}, zag_magnitude:{}", self.zag_magnitude);
                let rv = Some(Ok(ZagIterator::<T>::new(
                    start_pos,
                    zag_exit,
                    self.zag_vector,
                    n_zag_limit,
                )));

                // Move Outside
                // Move away from hull and one step in zig direction
                self.current_pos =
                    zag_exit + (self.zag_ray * self.safe_distance).add(self.zig_vector);
                self.current_zigs += 1;

                self.zag_vector = self.zag_vector.neg(); // Flip zag direction
                self.zag_ray = -self.zag_ray;
                self.zag_entry = self
                    .hull
                    .closest_ray_intersection(self.zag_ray, self.current_pos);

                rv
            } else {
                println!(
                    "Id:{} Current: {:?} Start: {:?} zag:{:?}, zig:{:?}",
                    self.chunk_id,
                    self.current_pos,
                    self.start_point,
                    self.zag_vector,
                    self.zig_vector
                );
                println!(
                    "inside, nothing found at pos: {:?}, entry:{:?} ray_dir {:?}",
                    self.current_pos, entry, self.zag_ray
                );
                println!(
                    "ray zag: {:?}",
                    self.hull
                        .closest_ray_intersection(self.zag_ray, self.current_pos,)
                );
                println!(
                    "ray -zag: {:?}",
                    self.hull
                        .closest_ray_intersection(self.zag_ray.neg(), self.current_pos,)
                );
                println!(
                    "Ray dir:{:?} number of zigs {:?}",
                    self.zag_ray, self.current_zigs
                );
                println!("hull: {:?}", self.hull);
                Some(Err(HronnError::InternalError(
                    "Could not find convex hull".to_string(),
                )))
            };
        }
        None
    }
}

/// Does zag iterations over a chunk
#[derive(Debug)]
pub struct ZagIterator<T: GenericVector3> {
    start_pos: T::Vector2,
    end_pos: T::Vector2,
    zag_vector: T::Vector2,
    current_zag: u32,
    n_zag_limit: u32,
}

impl<T: GenericVector3> ZagIterator<T> {
    fn new(
        start_pos: T::Vector2,
        end_pos: T::Vector2,
        zag_vector: T::Vector2,
        n_zag_limit: u32,
    ) -> Self {
        //println!(
        //    "ZagIterator::new() start={:?}, end={:?} zag_v:{:?}, n_zag_limit:{}",
        //    start_pos, end_pos, zag_vector, n_zag_limit
        //);
        Self {
            zag_vector,
            start_pos,
            end_pos,
            current_zag: 0,
            n_zag_limit,
        }
    }
}
impl<T: GenericVector3> Iterator for ZagIterator<T>
where
    u32: AsPrimitive<T::Scalar>,
{
    type Item = T::Vector2;

    fn next(&mut self) -> Option<Self::Item> {
        match self.current_zag {
            // Directly return None if n_zags is 0
            _ if self.n_zag_limit == 0 => None,
            // Return start_pos on the first iteration
            0 => {
                self.current_zag += 1;
                Some(self.start_pos)
            }
            // Return end_pos for the final iteration (or for n_zag_limit == 2)
            _ if self.current_zag == self.n_zag_limit - 1 => {
                self.current_zag += 1;
                Some(self.end_pos)
            }
            // For n_zag_limit > 2, return the zagged position for intermediate iterations
            _ if self.current_zag < self.n_zag_limit - 1 => {
                let result = self.start_pos + self.zag_vector * self.current_zag.as_();
                self.current_zag += 1;
                Some(result)
            }
            // For all other cases, return None
            _ => None,
        }
    }
}
