// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

//! # Hronn Crate
//!
//! `hronn` is a Rust crate containing an experimental CNC tool path generator/mesh sampler.
//! Work in progress.
//!

#![deny(
    rust_2018_compatibility,
    rust_2018_idioms,
    nonstandard_style,
    unused,
    future_incompatible,
    non_camel_case_types,
    unused_parens,
    non_upper_case_globals,
    unused_qualifications,
    unused_results,
    unused_imports,
    unused_variables,
    bare_trait_objects,
    ellipsis_inclusive_range_patterns,
    elided_lifetimes_in_paths
)]
#![warn(clippy::explicit_into_iter_loop)]

use crate::prelude::ConvertTo;
use linestring::linestring_2d::{convex_hull, Aabb2};
use vector_traits::num_traits::real::Real;
use vector_traits::{GenericScalar, GenericVector2, GenericVector3, HasXYZ};

mod geo;
mod impls;
mod meshanalyzer;
mod obj;
mod probe;
mod searchpattern;
mod strategyresult;
mod triangulation;
mod util;

pub mod prelude {
    pub use crate::{
        generate_aabb_then_convex_hull, generate_convex_hull_then_aabb,
        geo::{Circle, ConvertTo, RigidTransform2D},
        meshanalyzer::{MeshAnalyzer, MeshAnalyzerBuilder, SearchResult},
        obj::Obj,
        probe::{functions::ball_nose_to_edge_collision, SkipEndpoint},
        probe::{BallNoseProbe, Probe, SquareEndProbe},
        searchpattern::{
            meanderpattern::MeanderPattern, triangulatepattern::TriangulatePattern,
            AdaptiveSearchConfig, SearchPattern, SearchPatternConfig,
        },
        strategyresult::{LineData, MeshData, StrategyResult},
        triangulation::triangulate_vertices,
        util::MaximumTracker,
        HronnError, ProbeMode,
    };
}

// todo: replace with the approx numbers
pub const EPSILON: f64 = 1e-10;

#[derive(thiserror::Error, Debug)]
pub enum HronnError {
    #[error(transparent)]
    KrakelErrr(#[from] krakel::KrakelError),

    #[error(transparent)]
    LinestringError(#[from] linestring::LinestringError),

    #[error(transparent)]
    SpaceInsertionError(#[from] spade::InsertionError),

    #[error("Could not parse float value.")]
    ParseFloatError,

    #[error("The vertex indices does not match {0}")]
    MismatchedIndex(String),

    #[error("Your line-strings are self-intersecting: {0}")]
    SelfIntersectingData(String),

    #[error("The input data is not 2D: {0}")]
    InputNotPLane(String),

    #[error("Invalid data: {0}")]
    InvalidData(String),

    #[error("Aabb error: {0}")]
    AabbError(String),

    #[error("Transform error: {0}")]
    TransformError(String),

    #[error("Invalid input data: {0}")]
    InvalidParameter(String),

    #[error("Missing input data: {0}")]
    NoData(String),

    #[error("Obj file not triangulated: {0}")]
    NotTriangulated(String),

    #[error("Missing parameter: {0}")]
    MissingParameter(String),

    #[error("Mismatched MeshAnalyzer: {0}")]
    Mismatch(String),

    #[error("Unknown error: {0}")]
    InternalError(String),

    #[error(transparent)]
    IoError(#[from] std::io::Error),
}

#[cfg(test)]
mod tests {}

#[derive(PartialEq, Copy, Clone, Debug)]
pub enum ProbeMode {
    SplitTriangles,
    OriginalTriangles,
    BruteForce,
}

#[inline(always)]
pub fn m_factor_from_plane_unit_normal<T: GenericVector3>(unit_norm: T) -> T::Scalar {
    let m_sq = (unit_norm.x().powi(2) + unit_norm.y().powi(2)) / unit_norm.z().abs().powi(2);
    (T::Scalar::ONE + m_sq).sqrt()
}

#[inline(always)]
pub fn m_from_plane_unit_normal<T: GenericVector3>(unit_vector: T) -> T::Scalar {
    (unit_vector.x().powi(2) + unit_vector.y().powi(2)).sqrt() / unit_vector.z().abs()
}

#[derive(PartialEq, PartialOrd, Debug, Clone)]
struct GradientAndT {
    gradient: f64,
    t: f64,
}

/// Checks if a given point `q` is inside the 2D triangle defined by `p0`, `p1`, and `p2`.
///
/// The function determines the orientation of point `q` with respect to each edge of the triangle.
/// If `q` has the same orientation (either all counterclockwise or all clockwise) with respect to
/// each edge, it is considered inside the triangle.
///
/// # Parameters
/// - `q`: The point to check.
/// - `p0`: The first vertex of the triangle.
/// - `p1`: The second vertex of the triangle.
/// - `p2`: The third vertex of the triangle.
///
/// # Returns
/// - Returns `true` if `q` is inside the triangle, otherwise returns `false`.
///
/// # Examples
///
/// ```rust,ignore
/// let p0 = Vec2::new(0.0, 0.0);
/// let p1 = Vec2::new(1.0, 0.0);
/// let p2 = Vec2::new(0.0, 1.0);
///
/// assert!(is_inside_2d_triangle(Vec2::new(0.5, 0.5), p0, p1, p2));
/// ```
// todo: create a CCW specialization of this
#[inline(always)]
fn is_inside_2d_triangle<T: GenericVector2>(q: T, p0: T, p1: T, p2: T) -> bool {
    let orientation01 = (p1 - p0).perp_dot(q - p0);
    let orientation12 = (p2 - p1).perp_dot(q - p1);
    let orientation20 = (p0 - p2).perp_dot(q - p2);

    // All should have the same sign for the point to be inside
    orientation01 >= T::Scalar::ZERO
        && orientation12 >= T::Scalar::ZERO
        && orientation20 >= T::Scalar::ZERO
        || (orientation01 <= T::Scalar::ZERO
            && orientation12 <= T::Scalar::ZERO
            && orientation20 <= T::Scalar::ZERO)
}

/// Returns a non-normalized normal of this triangle, adjusted so that Z always points up
pub fn triangle_normal<T>(p0: T, p1: T, p2: T) -> T
where
    T: GenericVector3,
{
    let n = (p1.sub(p0)).cross(p2.sub(p0));
    // Ensure the normal points "upwards"
    if n.z() < T::Scalar::ZERO {
        n.neg()
    } else {
        n
    }
}

/// Build a convex hull from the point cloud, then build an AABB from that.
pub fn generate_convex_hull_then_aabb<T: GenericVector2, MESH: HasXYZ>(
    point_cloud: &[MESH],
) -> Result<(Aabb2<T>, Vec<T>), HronnError>
where
    MESH: ConvertTo<T::Vector3>,
{
    let mut aabb = Aabb2::default();

    let point_cloud: Vec<T> = point_cloud
        .iter()
        .map(|v| {
            // strip the Z coordinate off the bounding shape
            let v = v.to().to_2d();
            aabb.update_with_point(v);
            v
        })
        .collect();
    let convex_hull = convex_hull::graham_scan(&point_cloud)?;
    Ok((aabb, convex_hull))
}

/// Build an AABB from the point cloud, then build a convex hull from that aabb.
pub fn generate_aabb_then_convex_hull<T: GenericVector2, MESH: HasXYZ>(
    point_cloud: &[MESH],
) -> Result<(Aabb2<T>, Vec<T>), HronnError>
where
    MESH: ConvertTo<T::Vector3>,
{
    let mut aabb = Aabb2::default();
    for v in point_cloud {
        // strip the Z coordinate off the bounding shape
        let v = v.to().to_2d();
        aabb.update_with_point(v);
    }
    Ok((
        aabb,
        aabb.convex_hull()
            .ok_or_else(|| HronnError::InvalidData("No data found for convex hull".to_string()))?,
    ))
}
