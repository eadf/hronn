// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

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

use crate::prelude::{ConvertTo, MaximumTracker};
use ahash::AHashSet;
use linestring::linestring_2d::{convex_hull, Aabb2, LineString2};
use smallvec::SmallVec;
use std::cmp::Reverse;
use std::collections::HashMap;
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

/// Constructs a continuous loop of vertex indices from an unordered list of edges.
///
/// This function takes as input a slice of `usize` that represents edges by pairing
/// consecutive values. For example, a slice `[a, b, c, d]` represents two edges: `a-b` and `c-d`.
///
/// # Arguments
///
/// * `edges` - A slice of vertex indices, where each consecutive pair represents an edge.
///             The slice's length should be even.
///
/// # Returns
///
/// * If successful, a vector of vertex indices that forms a continuous loop.
/// * If unsuccessful, a `CollisionError` indicating the nature of the error.
///
/// # Example
///
/// ```rust,ignore
/// let edges = [1, 0, 2, 1, 3, 2, 0, 3];
/// let loop_indices = continuous_loop_from_unordered_edges(&edges)?;
/// assert_eq!(loop_indices, vec![1, 0, 3, 2, 1]);
/// ```
///
/// # Errors
///
/// This function may return an error in the following scenarios:
///
/// * The input edge list is malformed or does not form a valid loop.
/// * There are missing vertices in the adjacency map.
///
/// # Note
///
/// The function assumes that the input edge list is valid, i.e., forms a closed loop
/// without isolated vertices or unconnected components.
pub fn reconstruct_from_unordered_edges(edges: &[usize]) -> Result<Vec<usize>, HronnError> {
    let mut lowest_index = MaximumTracker::<Reverse<usize>>::default();

    if edges.len() < 2 {
        return Err(HronnError::InvalidParameter(
            "The line segment should have at least 2 vertices.".to_string(),
        ));
    }

    let mut adjacency: HashMap<usize, SmallVec<[usize; 2]>> = HashMap::new();
    for chunk in edges.chunks(2) {
        let a = chunk[0];
        let b = chunk[1];
        lowest_index.insert(Reverse(a));
        lowest_index.insert(Reverse(b));

        adjacency.entry(a).or_default().push(b);
        adjacency.entry(b).or_default().push(a);

        // Check for more than two neighbors and handle error
        if adjacency.get(&a).unwrap().len() > 2 || adjacency.get(&b).unwrap().len() > 2 {
            return Err(HronnError::InvalidParameter(
                "More than two neighbors for a vertex in a loop.".to_string(),
            ));
        }
    }

    // Detect endpoints (vertices with only one neighbor)
    let endpoints: Vec<_> = adjacency
        .iter()
        .filter(|(_, neighbors)| neighbors.len() == 1)
        .map(|(&vertex, _)| vertex)
        .collect();

    let is_loop = endpoints.is_empty();

    let mut current = if is_loop {
        // Start at lowest index for a loop
        lowest_index.get_max().unwrap().0
    } else {
        // Start at one of the endpoints for a line
        endpoints[0].min(endpoints[1])
    };
    let starting_point = current;

    let mut visited = AHashSet::new();
    let _ = visited.insert(current);
    let mut reconstructed = vec![current];

    let next_neighbors = &adjacency[&current];
    if (is_loop && next_neighbors.len() != 2) || (!is_loop && next_neighbors.len() > 1) {
        return Err(HronnError::InvalidParameter(
            "The provided line segment has more than two adjacent vertices.".to_string(),
        ));
    }

    if is_loop {
        current = next_neighbors[0].min(next_neighbors[1]);
    } else {
        current = next_neighbors[0]
    }
    reconstructed.push(current);
    let _ = visited.insert(current);
    loop {
        let next_neighbors: Vec<_> = adjacency[&current]
            .iter()
            .filter(|&n| !visited.contains(n))
            .collect();

        // Exit conditions
        if next_neighbors.is_empty() {
            break;
        }

        if next_neighbors.len() > 1 {
            return Err(HronnError::InvalidParameter(
                "The provided line segment have more than two adjacent vertices.".to_string(),
            ));
        }

        current = *next_neighbors[0];
        reconstructed.push(current);
        let _ = visited.insert(current);
    }
    // Add the starting point for a loop after the while loop.
    if is_loop {
        reconstructed.push(starting_point);
    }

    Ok(reconstructed)
}

/// Build a convex hull from the point cloud, then build an AABB from that.
pub fn generate_convex_hull_then_aabb<T: GenericVector2, MESH: HasXYZ>(
    point_cloud: &[MESH],
) -> Result<(Aabb2<T>, LineString2<T>), HronnError>
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
    let convex_hull = convex_hull::graham_scan(&point_cloud);
    Ok((aabb, convex_hull))
}

/// Build an AABB from the point cloud, then build a convex hull from that aabb.
pub fn generate_aabb_then_convex_hull<T: GenericVector2, MESH: HasXYZ>(
    point_cloud: &[MESH],
) -> Result<(Aabb2<T>, LineString2<T>), HronnError>
where
    MESH: ConvertTo<T::Vector3>,
{
    let mut aabb = Aabb2::default();
    for v in point_cloud {
        // strip the Z coordinate off the bounding shape
        let v = v.to().to_2d();
        aabb.update_with_point(v);
    }
    let convex_hull = LineString2(aabb.convex_hull().unwrap());
    Ok((aabb, convex_hull))
}
