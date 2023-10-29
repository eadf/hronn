// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.
pub(crate) mod functions;
#[cfg(test)]
mod tests;

use super::meshanalyzer::MeshAnalyzer;
use crate::{
    geo::{Area2D, ConvertTo, PlaneFromTriangle},
    m_factor_from_plane_unit_normal, m_from_plane_unit_normal,
    meshanalyzer::SearchResult,
    triangle_normal,
    util::MaximumTracker,
    HronnError,
};
use std::{fmt::Debug, marker::PhantomData};
use vector_traits::{
    num_traits::real::Real, GenericScalar, GenericVector2, GenericVector3, HasXYZ,
};

/// this struct contains pre-computed data needed to probe a triangle.
/// If the triangle has plane not perpendicular to the Z axis, the plane data will be populated.
pub struct TriangleMetadata<T: GenericVector3, MESH: HasXYZ> {
    pub delta_z: T::Scalar,
    pub plane: Option<PlaneMetadata<T::Vector2>>,
    #[doc(hidden)]
    _pdm: PhantomData<MESH>,
}

impl<T: GenericVector3, MESH: HasXYZ> TriangleMetadata<T, MESH>
where
    MESH: ConvertTo<T>,
{
    pub fn new_for_ball_nose(probe_radius: T::Scalar, p0: MESH, p1: MESH, p2: MESH) -> Self {
        let p0: T = p0.to();
        let p1: T = p1.to();
        let p2: T = p2.to();

        let u_normal = triangle_normal(p0, p1, p2).safe_normalize().unwrap();
        let m_factor = m_factor_from_plane_unit_normal::<T>(u_normal);

        let (p0_prim_xy, p1_prim_xy, p2_prim_xy) = {
            let xy_offset = (u_normal * probe_radius).to_2d();
            (
                p0.to_2d() + xy_offset,
                p1.to_2d() + xy_offset,
                p2.to_2d() + xy_offset,
            )
        };

        let area = Area2D::new(p0_prim_xy, p1_prim_xy, p2_prim_xy);
        let pft = PlaneFromTriangle::new_from_normal(u_normal, p0);
        if area.value().abs() > T::Scalar::EPSILON {
            Self {
                delta_z: m_factor * probe_radius,
                plane: Some(PlaneMetadata::<T::Vector2> {
                    pft,
                    translated_triangle: [p0_prim_xy, p1_prim_xy, p2_prim_xy],
                }),
                _pdm: PhantomData,
            }
        } else {
            Self {
                delta_z: m_factor * probe_radius,
                plane: None,
                _pdm: PhantomData,
            }
        }
    }

    pub fn new_for_square_end(probe_radius: T::Scalar, p0: T, p1: T, p2: T) -> Self {
        let normal = triangle_normal::<T>(p0, p1, p2);
        let u_normal_3d = normal.safe_normalize().unwrap();
        let m = m_from_plane_unit_normal::<T>(u_normal_3d);
        let p0_2d = p0.to_2d();
        let p1_2d = p1.to_2d();
        let p2_2d = p2.to_2d();

        if let Some(u_normal_2d) = normal.to_2d().safe_normalize() {
            // the area of a translated triangle remains the same
            let area = Area2D::new(p0_2d, p1_2d, p2_2d);
            if area.value().abs() > T::Scalar::EPSILON {
                let xy_offset = u_normal_2d * probe_radius;
                let (p0_prim_xy, p1_prim_xy, p2_prim_xy) =
                    (p0_2d + xy_offset, p1_2d + xy_offset, p2_2d + xy_offset);
                let pft = PlaneFromTriangle::new_from_normal(u_normal_3d, p0);
                return Self {
                    delta_z: m * probe_radius,
                    plane: Some(PlaneMetadata {
                        pft,
                        translated_triangle: [p0_prim_xy, p1_prim_xy, p2_prim_xy],
                    }),
                    _pdm: PhantomData,
                };
            }
        } else {
            //println!("The triangle {p0} {p1} {p2} had no suitable plane. norm:{}", normal.xy());
            let area = Area2D::new(p0_2d, p1_2d, p2_2d);
            if area.value().abs() > T::Scalar::EPSILON {
                return Self {
                    delta_z: m * probe_radius,
                    plane: Some(PlaneMetadata {
                        pft: PlaneFromTriangle::new_from_z_coord(p0.z()),
                        translated_triangle: [p0_2d, p1_2d, p2_2d],
                    }),
                    _pdm: PhantomData,
                };
            }
        }
        Self {
            delta_z: m * probe_radius,
            plane: None,
            _pdm: PhantomData,
        }
    }
}

/// This structure contains the pre-computed data needed to process a sphere collision vs triangle
pub struct PlaneMetadata<T: GenericVector2> {
    pub pft: PlaneFromTriangle<T>,
    pub translated_triangle: [T; 3],
}

/// Parameters needed to query the kd-tree
pub struct QueryParameters<'a, T: GenericVector3, MESH: HasXYZ> {
    pub(crate) vertices: &'a [MESH],
    pub(crate) indices: &'a [usize],
    pub(crate) meta_data: &'a [TriangleMetadata<T, MESH>],
    pub(crate) probe_radius: T::Scalar,
    pub(crate) search_radius: T::Scalar,
}

pub trait Probe<T: GenericVector3, MESH: HasXYZ + ConvertTo<T>> {
    fn probe_radius(&self) -> T::Scalar;

    /// Internal use only.
    #[doc(hidden)]
    fn _mesh_analyzer(&self) -> &MeshAnalyzer<'_, T, MESH>;

    /// Internal use only.
    #[doc(hidden)]
    fn _meta_data(&self) -> &[TriangleMetadata<T, MESH>];

    /// Internal use only.
    #[doc(hidden)]
    #[allow(clippy::type_complexity)]
    fn _collision_fn(
        &self,
    ) -> fn(
        query_parameters: &QueryParameters<'_, T, MESH>,
        site_index: usize,
        center: T::Vector2,
        mt: &mut MaximumTracker<SearchResult<T>>,
    );
}

pub struct SquareEndProbe<'b, T: GenericVector3, MESH: HasXYZ>
where
    MESH: ConvertTo<T>,
{
    probe_radius: T::Scalar,
    meta_data: Vec<TriangleMetadata<T, MESH>>,
    bound_mesh_analyzer: &'b MeshAnalyzer<'b, T, MESH>,
}

impl<'b, T: GenericVector3, MESH: HasXYZ> SquareEndProbe<'b, T, MESH>
where
    MESH: ConvertTo<T>,
{
    pub fn new(
        mesh_analyzer: &'b MeshAnalyzer<'_, T, MESH>,
        probe_radius: T::Scalar,
    ) -> Result<Self, HronnError> {
        Ok(SquareEndProbe {
            probe_radius,
            meta_data: functions::shared_square_end_precompute_logic::<T, MESH>(
                mesh_analyzer.vertices.as_ref(),
                mesh_analyzer.indices.as_ref(),
                probe_radius,
            ),
            bound_mesh_analyzer: mesh_analyzer,
        })
    }
}

impl<'b, T: GenericVector3, MESH: HasXYZ> Probe<T, MESH> for SquareEndProbe<'b, T, MESH>
where
    MESH: ConvertTo<T>,
{
    fn probe_radius(&self) -> T::Scalar {
        self.probe_radius
    }

    fn _mesh_analyzer(&self) -> &MeshAnalyzer<'_, T, MESH> {
        self.bound_mesh_analyzer
    }

    fn _meta_data(&self) -> &[TriangleMetadata<T, MESH>] {
        &self.meta_data
    }

    fn _collision_fn(
        &self,
    ) -> fn(
        query_parameters: &QueryParameters<'_, T, MESH>,
        site_index: usize,
        center: T::Vector2,
        mt: &mut MaximumTracker<SearchResult<T>>,
    ) {
        functions::square_end_compute_collision
    }
}

pub struct BallNoseProbe<'b, T: GenericVector3, MESH: HasXYZ>
where
    MESH: ConvertTo<T>,
{
    probe_radius: T::Scalar,
    meta_data: Vec<TriangleMetadata<T, MESH>>,
    bound_mesh_analyzer: &'b MeshAnalyzer<'b, T, MESH>,
}

impl<'b, T: GenericVector3, MESH: HasXYZ> BallNoseProbe<'b, T, MESH>
where
    MESH: ConvertTo<T>,
{
    pub fn new(
        mesh_analyzer: &'b MeshAnalyzer<'_, T, MESH>,
        probe_radius: T::Scalar,
    ) -> Result<Self, HronnError> {
        Ok(BallNoseProbe {
            probe_radius,
            meta_data: functions::shared_ball_nose_precompute_logic::<T, MESH>(
                mesh_analyzer.vertices.as_ref(),
                mesh_analyzer.indices.as_ref(),
                probe_radius,
            ),
            bound_mesh_analyzer: mesh_analyzer,
        })
    }
}

impl<'b, T: GenericVector3, MESH: HasXYZ> Probe<T, MESH> for BallNoseProbe<'b, T, MESH>
where
    MESH: ConvertTo<T>,
    T: ConvertTo<MESH>,
{
    fn probe_radius(&self) -> T::Scalar {
        self.probe_radius
    }

    fn _mesh_analyzer(&self) -> &MeshAnalyzer<'_, T, MESH> {
        self.bound_mesh_analyzer
    }

    fn _meta_data(&self) -> &[TriangleMetadata<T, MESH>] {
        &self.meta_data
    }

    fn _collision_fn(
        &self,
    ) -> fn(
        query_parameters: &QueryParameters<'_, T, MESH>,
        site_index: usize,
        center: T::Vector2,
        mt: &mut MaximumTracker<SearchResult<T>>,
    ) {
        functions::ball_nose_compute_collision
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(i8)]
#[allow(dead_code)]
pub enum SkipEndpoint {
    SkipP0 = -1,
    NoSkip = 0,
    SkipP1 = 1,
}

impl SkipEndpoint {
    pub(crate) fn flip(self) -> Self {
        // Multiply by -1 to flip the sign
        unsafe { std::mem::transmute::<i8, Self>(-(self as i8)) }
    }
}
