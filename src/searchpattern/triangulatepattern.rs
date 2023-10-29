// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use super::SearchPatternImpl;
use crate::{
    meshanalyzer::MeshAnalyzer,
    prelude::*,
    probe::QueryParameters,
    searchpattern::{
        adaptivesamplebuffer::AdaptiveSampleBuffer, meanderpath::ConvexHullMeanderPath,
        SearchPatternConfig,
    },
    triangulation::{DelaunayContainer, DelaunayPos},
    HronnError, ProbeMode,
};
use krakel::PointTrait;
use linestring::linestring_2d::{Aabb2, LineString2};
#[allow(unused_imports)]
use rayon::prelude::*;
use spade::{DelaunayTriangulation, Triangulation};
use std::marker::PhantomData;
use vector_traits::{
    num_traits::{real::Real, AsPrimitive},
    Approx, GenericScalar, GenericVector2, GenericVector3, HasXY, HasXYZ,
};
#[derive(Default)]
pub struct TriangulatePattern<T: GenericVector3, MESH: HasXYZ>
where
    MESH: ConvertTo<T>,
{
    aabb: Option<Aabb2<T::Vector2>>,
    convex_hull: Option<LineString2<T::Vector2>>,
    step: Option<T::Scalar>,
    #[doc(hidden)]
    _pd: PhantomData<MESH>,
}

impl<T: GenericVector3, MESH: HasXYZ> TriangulatePattern<T, MESH>
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

impl<T: GenericVector3, MESH: HasXYZ> SearchPattern<T, MESH> for TriangulatePattern<T, MESH>
where
    T::Vector2: PointTrait<PScalar = T::Scalar>,
    T: ConvertTo<MESH>,
    MESH: ConvertTo<T> + Approx,
    u32: AsPrimitive<MESH::Scalar>,
    u32: AsPrimitive<T::Scalar>,
    T::Scalar: AsPrimitive<MESH::Scalar>,
{
    /// The error handling implementation of the triangulate strategy pattern.
    /// This method is designed to ony use the dynamic dispatch of the &dyn Probe a hand full of
    /// times during the entire search() operation.
    fn search(
        &self,
        context: &MeshAnalyzer<'_, T, MESH>,
        config: &SearchPatternConfig<'_, T, MESH>,
    ) -> Result<StrategyResult<MESH>, HronnError> {
        if config.probe._mesh_analyzer() != context {
            return Err(HronnError::Mismatch("".to_string()));
        }
        if config.probe._mesh_analyzer() != context {
            return Err(HronnError::Mismatch("".to_string()));
        }
        if self.convex_hull.is_none() {
            return Err(HronnError::MissingParameter("convex_hull".to_string()));
        }
        if self.step.is_none() {
            return Err(HronnError::MissingParameter("step".to_string()));
        }
        if self.step.is_none() {
            return Err(HronnError::MissingParameter("step".to_string()));
        }
        self.search_impl(context, config)
    }
}

impl<T: GenericVector3, MESH: HasXYZ> SearchPatternImpl<T, MESH> for TriangulatePattern<T, MESH>
where
    T::Vector2: PointTrait<PScalar = T::Scalar>,
    T: ConvertTo<MESH>,
    MESH: ConvertTo<T> + Approx,
    T::Scalar: AsPrimitive<MESH::Scalar>,
    u32: AsPrimitive<MESH::Scalar>,
    u32: AsPrimitive<T::Scalar>,
{
    /// The actual implementation of the triangulate strategy pattern.
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
        let minimum_z = config.minimum_z;
        let step = self.step.unwrap();

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

        let aabb = self.aabb.unwrap();
        //println!("aabb = {:?}", aabb);
        //println!("convex hull = {:?}", convex_hull);

        /*Aabb2::new(*convex_hull.0.first().unwrap());
        for p in convex_hull.0.iter() {
            aabb.update_with_point(*p);
        }*/

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
        let result = result?;

        // sample along the convex hull
        let convex_hull: Vec<MESH> = convex_hull
            .0
            .iter()
            .map(|v| {
                let r = search_fn(self, context, *v, &qp, collision_fn)
                    .map(|z| z.z_value.max(minimum_z))
                    .unwrap_or(minimum_z);
                v.to_3d(r).to()
            })
            .collect();
        let (vertices, indices) =
            self.triangulate_meandering_vertices(aabb, convex_hull, result)?;

        Ok(StrategyResult::MeshData(MeshData { vertices, indices }))
    }
}

impl<T: GenericVector3, MESH: HasXYZ> TriangulatePattern<T, MESH>
where
    T::Vector2: PointTrait<PScalar = T::Scalar>,
    T: ConvertTo<MESH>,
    MESH: ConvertTo<T>,
    T::Scalar: AsPrimitive<MESH::Scalar>,
{
    fn triangulate_meandering_vertices(
        &self,
        aabb2: Aabb2<T::Vector2>,
        hull_3d: Vec<MESH>,
        vertices: Vec<Vec<MESH>>,
    ) -> Result<(Vec<MESH>, Vec<usize>), HronnError> {
        if let Some((min, max, width, height)) = aabb2.extents() {
            let min = min.to_3d(T::Scalar::ZERO).to();
            let max = max.to_3d(T::Scalar::ZERO).to();
            let width: MESH::Scalar = width.as_();
            let height: MESH::Scalar = height.as_();

            let width = width * (20.0.into());
            let height = height * (20.0.into());
            let big_triangle_a =
                MESH::new_3d(min.x() - width, min.y() - height, MESH::Scalar::ZERO);
            let big_triangle_b =
                MESH::new_3d(max.x() + width, min.y() - height, MESH::Scalar::ZERO);
            let big_triangle_c = MESH::new_3d(
                min.x() + width / MESH::Scalar::TWO,
                max.y() - height,
                MESH::Scalar::ZERO,
            );
            println!(
                "Big triangle: {:?},{:?},{:?},",
                big_triangle_a, big_triangle_b, big_triangle_c
            );
            let mut tris = DelaunayContainer {
                delaunay: DelaunayTriangulation::<DelaunayPos<MESH>>::new(),
                vertices: Vec::<MESH>::with_capacity(4 + hull_3d.len() + vertices.len()),
                indices: Vec::<usize>::with_capacity((4 + hull_3d.len() + vertices.len()) * 3),
            };

            let mut last_vertex = tris.insert_vertex(big_triangle_a)?;
            last_vertex = tris.insert_vertex_with_hint(big_triangle_b, last_vertex)?;
            let _ = tris.insert_vertex_with_hint(big_triangle_c, last_vertex)?;

            last_vertex = tris.insert_vertex(*hull_3d.first().unwrap())?;
            for v in hull_3d.iter().skip(1) {
                last_vertex = tris.insert_vertex_with_hint(*v, last_vertex)?;
            }
            //println!("hull: {:?}", hull_3d);
            for v in vertices.iter() {
                for vv in v {
                    last_vertex = tris.insert_vertex_with_hint(*vv, last_vertex)?;
                }
            }

            for face in tris.delaunay.inner_faces() {
                // face is a FaceHandle
                // edges is an array containing 3 directed edge handles
                /*let edges = face.adjacent_edges();
                for edge in &edges {
                    let from = edge.from();
                    let to = edge.to();
                    // from and to are vertex handles
                    println!("found an edge: {:?} -> {:?}", from, to);
                }*/

                // vertices is an array containing 3 vertex handles
                let vertices = face.vertices();
                let id0 = vertices[0].fix().index();
                let id1 = vertices[1].fix().index();
                let id2 = vertices[2].fix().index();

                if id0 > 2 && id1 > 2 && id2 > 2 {
                    let id0 = id0 - 3;
                    let id1 = id1 - 3;
                    let id2 = id2 - 3;

                    /*println!(
                        "Found inside triangle {:?},{:?},{:?} {},{},{},",
                        tris.vertices[id0], tris.vertices[id1], tris.vertices[id2], id0, id1, id2
                    );*/
                    tris.indices.push(id0);
                    tris.indices.push(id1);
                    tris.indices.push(id2);
                } else {
                    /*let id0 = vertices[0].fix().index();
                    let id1 = vertices[1].fix().index();
                    let id2 = vertices[2].fix().index();
                    println!(
                        "Found outside triangle {:?},{:?},{:?} {},{},{},",
                        tris.vertices[id0], tris.vertices[id1], tris.vertices[id2], id0, id1, id2
                    );*/
                }
            }
            Ok((tris.vertices, tris.indices))
        } else {
            Ok((vec![], vec![]))
        }
    }
}
