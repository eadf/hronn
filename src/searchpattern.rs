// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.
mod adaptivesamplebuffer;
pub(crate) mod meanderpath;
pub mod meanderpattern;
pub mod triangulatepattern;

use super::{
    meshanalyzer::MeshAnalyzer,
    probe::{Probe, QueryParameters},
};
use crate::{meshanalyzer::SearchResult, prelude::*, HronnError};
use krakel::PointTrait;
use std::{cell::RefCell, collections::HashSet};
use vector_traits::{num_traits::real::Real, GenericScalar, GenericVector3, HasXYZ};
//use krakel::PointTrait;

#[derive(Clone, Copy, Debug)]
pub struct AdaptiveSearchConfig<T> {
    pub xy_sample_dist: T,
    pub z_jump_threshold: T,
    pub buffer_size: usize,
    pub reduce: bool,
}

impl<T: Real> AdaptiveSearchConfig<T> {
    pub fn new(xy_sample_dist: T, z_jump_threshold: T, reduce: bool) -> Self {
        Self {
            xy_sample_dist,
            z_jump_threshold,
            buffer_size: 5,
            reduce,
        }
    }
}

pub struct SearchPatternConfig<'a, T: GenericVector3, MESH: HasXYZ + ConvertTo<T>> {
    pub probe: &'a dyn Probe<T, MESH>,
    pub minimum_z: T::Scalar,
    pub adaptive_mesh_config: Option<AdaptiveSearchConfig<T::Scalar>>,
}

impl<'a, T: GenericVector3, MESH: HasXYZ + ConvertTo<T>> SearchPatternConfig<'a, T, MESH> {
    pub fn new(probe: &'a dyn Probe<T, MESH>, minimum_z: T::Scalar) -> Self {
        Self {
            probe,
            minimum_z,
            adaptive_mesh_config: None,
        }
    }
    pub fn with_adaptive_config(mut self, adaptive: AdaptiveSearchConfig<T::Scalar>) -> Self {
        self.adaptive_mesh_config = Some(adaptive);
        self
    }
}

/// A Strategy interface
pub trait SearchPattern<T: GenericVector3, MESH: HasXYZ + ConvertTo<T>> {
    /// This method is supposed to only use the dynamic dispatch of the &dyn Probe a hand full of
    /// times during the entire search() operation.
    fn search(
        &self,
        context: &MeshAnalyzer<'_, T, MESH>,
        config: &SearchPatternConfig<'_, T, MESH>,
    ) -> Result<StrategyResult<MESH>, HronnError>;
}

pub(crate) trait SearchPatternImpl<T: GenericVector3, MESH: HasXYZ>
where
    T::Vector2: PointTrait<PScalar = T::Scalar>,
    MESH: ConvertTo<T>,
{
    /// This method is supposed to only use the dynamic dispatch of the &dyn Probe a hand full of
    /// times during the entire search() operation.
    fn search_impl(
        &self,
        context: &MeshAnalyzer<'_, T, MESH>,
        config: &SearchPatternConfig<'_, T, MESH>,
    ) -> Result<StrategyResult<MESH>, HronnError>;

    /// Performs a query using the original (non-split) mesh.
    ///
    /// This function serves as the "inner loop" for collision computations.
    /// To achieve optimal performance, this function does not use any dynamic dispatch.
    ///
    /// # Parameters
    ///
    /// * `center`: The center point of the probe.
    /// * `search_radius`: The radius around the `center` in which the search is performed.
    /// * `probe`: The probe object responsible for computing collisions.
    ///
    /// # Returns
    ///
    /// * An `Option<T::Scalar>` that represents the maximum value computed by the
    /// `probe.compute_collision()` function. If no collisions are detected,
    /// it returns `None`.
    #[allow(clippy::type_complexity)]
    fn query_nonsplit_point(
        &self,
        ma: &MeshAnalyzer<'_, T, MESH>,
        center: T::Vector2,
        qp: &QueryParameters<'_, T, MESH>,
        collision_fn: fn(
            query_parameters: &QueryParameters<'_, T, MESH>,
            site_index: usize,
            center: T::Vector2,
            mt: &mut MaximumTracker<SearchResult<T>>,
        ),
    ) -> Option<SearchResult<T>> {
        let mut mt = MaximumTracker::<SearchResult<T>>::default();

        ma.kd_tree
            .closure_range_query(&center, qp.search_radius, |site| {
                collision_fn(qp, site.index, center, &mut mt)
            });

        mt.get_max()
    }

    /// Performs a query using the split mesh.
    ///
    /// This function serves as the "inner loop" for collision computations.
    /// To achieve optimal performance, this function does not use any dynamic dispatch.
    ///
    /// # Parameters
    ///
    /// * `center`: The center point of the probe.
    /// * `search_radius`: The radius around the `center` in which the search is performed.
    /// * `probe`: The probe object responsible for computing collisions.
    ///
    /// # Returns
    ///
    /// * An `Option<T::Scalar>` that represents the maximum value computed by the
    /// `probe.compute_collision()` function. If no collisions are detected,
    /// it returns `None`.
    #[allow(clippy::type_complexity)]
    fn query_split_point(
        &self,
        ma: &MeshAnalyzer<'_, T, MESH>,
        center: T::Vector2,
        qp: &QueryParameters<'_, T, MESH>,
        collision_fn: fn(
            query_parameters: &QueryParameters<'_, T, MESH>,
            site_index: usize,
            center: T::Vector2,
            mt: &mut MaximumTracker<SearchResult<T>>,
        ),
    ) -> Option<SearchResult<T>> {
        let mut mt = MaximumTracker::<SearchResult<T>>::default();

        // The original triangles are split, only report original triangles
        THREAD_LOCAL_SET.with(|set| {
            let mut already_tested = set.borrow_mut();
            already_tested.clear();
            ma.kd_tree
                .closure_range_query(&center, qp.search_radius, |site| {
                    let site_index = site.index;
                    if already_tested.insert(site_index) {
                        collision_fn(qp, site_index, center, &mut mt)
                        //probe.compute_collision(vertices, indices, site_index, &mut mt, center);
                    }
                });
            already_tested.clear();
        });
        mt.get_max()
    }

    /// a search query that bypasses the kd-tree and simply tests each and every triangle
    #[allow(clippy::type_complexity)]
    fn brute_force_query_point(
        &self,
        _: &MeshAnalyzer<'_, T, MESH>,
        center: T::Vector2,
        qp: &QueryParameters<'_, T, MESH>,
        collision_fn: fn(
            query_parameters: &QueryParameters<'_, T, MESH>,
            site_index: usize,
            center: T::Vector2,
            mt: &mut MaximumTracker<SearchResult<T>>,
        ),
    ) -> Option<SearchResult<T>> {
        let mut mt = MaximumTracker::<SearchResult<T>>::default();

        let site_index_max = qp.indices.len() / 3;
        for site_index in 0..site_index_max {
            // Brute force vertex collision processing
            collision_fn(qp, site_index, center, &mut mt);
        }
        mt.get_max()
    }

    /// The distance we use when searching the kd-tree
    /// We can search by a distance of r+l/2, i.e: 1.5*r if l=r
    #[inline(always)]
    fn calculate_search_radius(
        probe_radius: T::Scalar,
        triangle_side_length: T::Scalar,
    ) -> T::Scalar {
        probe_radius + triangle_side_length / T::Scalar::TWO
    }
}

// a thread local HashSet used by the query_split_point() method
// we use a thread local container so that we don't have to allocate it again and again.
// Remember to clear & shrink this after each search pattern is executed
thread_local! {
    static THREAD_LOCAL_SET: RefCell<HashSet<usize>> = RefCell::new(HashSet::new());
}
