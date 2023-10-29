// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use super::{adaptivesamplebuffer::AdaptiveSampleBuffer, SearchPatternImpl};
use crate::{
    meshanalyzer::MeshAnalyzer,
    prelude::*,
    probe::QueryParameters,
    searchpattern::{meanderpath::ConvexHullMeanderPath, SearchPatternConfig},
    HronnError, ProbeMode,
};
use krakel::PointTrait;
use linestring::linestring_2d::{Aabb2, LineString2};
#[allow(unused_imports)]
use rayon::iter::ParallelIterator;
use std::marker::PhantomData;
use vector_traits::{
    num_traits::{real::Real, AsPrimitive},
    Approx, GenericScalar, GenericVector2, GenericVector3, HasXY, HasXYZ,
};

#[derive(Default)]
pub struct MeanderPattern<T: GenericVector3, MESH: HasXYZ>
where
    MESH: ConvertTo<T>,
{
    convex_hull: Option<LineString2<T::Vector2>>,
    aabb: Option<Aabb2<T::Vector2>>,
    step: Option<T::Scalar>,
    #[doc(hidden)]
    _pd: PhantomData<MESH>,
}

impl<T: GenericVector3, MESH: HasXYZ> MeanderPattern<T, MESH>
where
    T::Vector2: PointTrait<PScalar = T::Scalar>,
    T: ConvertTo<MESH>,
    MESH: ConvertTo<T>,
{
    pub fn new(
        aabb: Aabb2<T::Vector2>,
        convex_hull: LineString2<T::Vector2>,
        step: T::Scalar,
    ) -> Result<Self, HronnError> {
        if let Some((min, max)) = aabb.min_max() {
            if min.distance_sq(max) < 0.0001.into() {
                return Err(HronnError::MissingParameter(
                    "aabb was too small".to_string(),
                ));
            }
        } else {
            return Err(HronnError::MissingParameter(
                "aabb was degenerate".to_string(),
            ));
        }
        Ok(Self {
            aabb: Some(aabb),
            convex_hull: Some(convex_hull),
            step: Some(step),
            _pd: PhantomData,
        })
    }

    pub fn convex_hull(mut self, convex_hull: LineString2<T::Vector2>) -> Result<Self, HronnError> {
        self.convex_hull = Some(convex_hull);
        Ok(self)
    }

    pub fn aabb(mut self, aabb: Aabb2<T::Vector2>) -> Result<Self, HronnError> {
        self.aabb = Some(aabb);
        Ok(self)
    }

    pub fn step(mut self, step: T::Scalar) -> Result<Self, HronnError> {
        self.step = Some(step);
        Ok(self)
    }
}

impl<T: GenericVector3, MESH: HasXYZ> SearchPattern<T, MESH> for MeanderPattern<T, MESH>
where
    T::Vector2: PointTrait<PScalar = T::Scalar>,
    MESH: ConvertTo<T> + Approx,
    u32: AsPrimitive<MESH::Scalar>,
    u32: AsPrimitive<T::Scalar>,
    T::Scalar: AsPrimitive<MESH::Scalar>,
    T: ConvertTo<MESH>,
{
    fn search(
        &self,
        context: &MeshAnalyzer<'_, T, MESH>,
        config: &SearchPatternConfig<'_, T, MESH>,
    ) -> Result<StrategyResult<MESH>, HronnError> {
        if config.probe._mesh_analyzer() != context {
            return Err(HronnError::Mismatch("".to_string()));
        }
        if self.aabb.is_none() {
            return Err(HronnError::MissingParameter("aabb".to_string()));
        }
        if self.convex_hull.is_none() {
            return Err(HronnError::MissingParameter("convex_hull".to_string()));
        }
        if self.step.is_none() {
            return Err(HronnError::MissingParameter("step".to_string()));
        }
        let rv = self.search_impl(context, config);
        // make sure the indices hash set is not allocating any memory
        super::THREAD_LOCAL_SET.with(|set| {
            let mut already_tested = set.borrow_mut();
            already_tested.clear();
            already_tested.shrink_to_fit();
        });
        rv
    }
}

impl<T: GenericVector3, MESH: HasXYZ> SearchPatternImpl<T, MESH> for MeanderPattern<T, MESH>
where
    T::Vector2: PointTrait<PScalar = T::Scalar>,
    T: ConvertTo<MESH>,
    MESH: ConvertTo<T> + Approx,
    T::Scalar: AsPrimitive<MESH::Scalar>,
    u32: AsPrimitive<MESH::Scalar>,
    u32: AsPrimitive<T::Scalar>,
{
    /// The actual implementation of the meander strategy pattern.
    /// This method is designed to ony use the dynamic dispatch of the &dyn Probe a hand full of
    /// times during the entire search() operation.
    fn search_impl(
        &self,
        context: &MeshAnalyzer<'_, T, MESH>,
        config: &SearchPatternConfig<'_, T, MESH>,
    ) -> Result<StrategyResult<MESH>, HronnError> {
        // all of these parameters should already been checked for None
        let search_radius = Self::calculate_search_radius(
            config.probe.probe_radius(),
            context.triangle_side_length,
        );
        let convex_hull = self.convex_hull.as_ref().unwrap();
        let aabb = self.aabb.unwrap();
        //println!("convex_hull:{:?}", convex_hull);
        //println!("aabb:{:?}", aabb);

        let minimum_z = config.minimum_z;
        let step = self.step.unwrap();

        let mut sample_data = LineData::<MESH>::default();
        let qp = {
            let ma = context;
            QueryParameters {
                vertices: ma.vertices.as_ref(),
                indices: ma.indices.as_ref(),
                meta_data: config.probe._meta_data(),
                probe_radius: config.probe.probe_radius(),
                search_radius,
            }
        };
        let collision_fn = config.probe._collision_fn();
        // todo: check that context == probe._mesh_analyzer();
        //let context = probe._mesh_analyzer();

        let search_fn = match context.mode {
            ProbeMode::BruteForce => {
                println!("Probing the mesh by brute force");
                Self::brute_force_query_point
            }
            ProbeMode::OriginalTriangles => Self::query_nonsplit_point,
            ProbeMode::SplitTriangles => Self::query_split_point,
        };

        let zag_unit_vector = T::Vector2::new_2d(T::Scalar::ONE, T::Scalar::ZERO);
        let zig_unit_vector = T::Vector2::new_2d(T::Scalar::ZERO, T::Scalar::ONE);

        let path = ConvexHullMeanderPath::<T, MESH>::new(
            convex_hull,
            aabb,
            aabb.low().unwrap(),
            zig_unit_vector,
            zag_unit_vector,
            step,
        );

        let result: Result<Vec<_>, HronnError> = if let Some(adaptive_mesh_config) =
            config.adaptive_mesh_config
        {
            println!(
                "config.adaptive_mesh_config:{:?}",
                config.adaptive_mesh_config
            );
            // we need to cache this or .into_par_iter() will complain about the probe &dyn
            let cached_reduce_setting = config.adaptive_mesh_config.unwrap().reduce;
            // the min distance in xy we allow for adaptive re-sampling, we stop sampling after this distance
            let xy_dist_criteria_sq = adaptive_mesh_config.xy_sample_dist.powi(2);
            // How steep inclination in Z we allow between two samples
            let z_dist_criteria = adaptive_mesh_config.z_jump_threshold;
            path.into_par_iter()?
                .map(|chunk| {
                    let mut buffer: AdaptiveSampleBuffer<T, MESH, _> = AdaptiveSampleBuffer::new(
                        adaptive_mesh_config.buffer_size,
                        xy_dist_criteria_sq,
                        z_dist_criteria,
                        |pos: T::Vector2| -> MESH {
                            let rv: Option<T> = search_fn(self, context, pos, &qp, collision_fn)
                                .map(|z| pos.to_3d(z.z_value.max(minimum_z)));
                            rv.unwrap_or_else(|| pos.to_3d(minimum_z)).to()
                        },
                    );
                    let buffer_fn = if cached_reduce_setting {
                        AdaptiveSampleBuffer::sample_and_refine_reduce
                    } else {
                        AdaptiveSampleBuffer::sample_and_refine
                    };
                    for zag_iter in chunk {
                        for pos in zag_iter? {
                            buffer_fn(&mut buffer, pos);
                        }
                        buffer.drain();
                    }
                    Ok(buffer.finalize())
                })
                .collect()
        } else {
            path.into_par_iter()?
                .map(|chunk| {
                    let mut result = Vec::<MESH>::new();
                    for zag_iter in chunk {
                        for pos in zag_iter? {
                            let rv = search_fn(self, context, pos, &qp, collision_fn)
                                .map(|z| pos.to_3d(z.z_value.max(minimum_z)));
                            result.push(rv.unwrap_or_else(|| pos.to_3d(minimum_z)).to());
                        }
                    }
                    Ok(result)
                })
                .collect()
        };
        //println!("result:{:?}", result?);

        for chunk in result? {
            for v in chunk {
                if v.x().is_finite() && v.y().is_finite() && v.z().is_finite() {
                    sample_data.continue_line(v);
                } else {
                    println!("++++ignoring abnormal value: {:?}", v);
                }
            }
        }

        Ok(StrategyResult::LineData(sample_data))
    }
}
