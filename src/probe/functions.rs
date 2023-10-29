// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use super::{PlaneMetadata, QueryParameters, SkipEndpoint, TriangleMetadata};
use crate::{
    geo::ConvertTo,
    prelude::*,
    {is_inside_2d_triangle, meshanalyzer::SearchResult},
};
use vector_traits::{
    approx, approx::ulps_eq, num_traits::real::Real, GenericScalar, GenericVector2, GenericVector3,
    HasXYZ,
};

/// Calculates the Z-coordinate where a 2d circle (radius in X&Y) intersects with an edge. The circle can be considered to
/// fall from infinite Z, so the effect is the same as for a cylinder or square end mill.
///
/// This function determines the Z-coordinate at which a circle, defined by its `center` and `probe_radius`,
/// intersects with a line segment defined by points `p0` and `p1`.
///
/// # Arguments
///
/// * `center`: The XY-coordinates of the center of the circle.
/// * `probe_radius`: The radius of the sphere.
/// * `p0`: One end of the line segment. The function internally re-arranges `p0` and `p1` based on their Z-coordinates,
///         so their order in the argument list is not significant.
/// * `p1`: The other end of the line segment.
/// * `skip`: An enum value indicating whether to skip calculations related to one of the endpoints of the line segment.
///
/// # Returns
///
/// * An `Option<f64>` representing the highest Z-coordinate of intersection. If there is no intersection, it returns `None`.
///
/// # Notes
///
/// * The function may internally swap `p0` and `p1` to ensure `p0` always has a Z-coordinate less than or equal to `p1`.
///   This means callers do not need to pre-sort these points before calling the function.
/// * The `skip` argument allows for efficient handling of cases where one of the endpoints' calculations can be avoided.
///
/// # Examples
///
/// ```rust,ignore
/// let center = Vector2::new(1.0, 2.0);
/// let probe_radius = 1.5;
/// let p0 = Vector3::new(0.0, 0.0, 0.0);
/// let p1 = Vector3::new(2.0, 2.0, 2.0);
/// let z_value = cylinder_to_edge_collision(center, probe_radius, p0, p1, SkipEndpoint::NoSkip);
/// ```
fn square_end_to_edge_collision<T: GenericVector3>(
    center: T::Vector2,
    probe_radius: T::Scalar,
    site_index: usize,
    mut p0: T,
    mut p1: T,
    mt: &mut MaximumTracker<SearchResult<T>>,
) where
    T::Scalar: approx::UlpsEq,
{
    if p0.z() > p1.z() {
        // p1.z is now always higher or equal to p1.z
        std::mem::swap(&mut p0, &mut p1);
    }
    let r_sq = probe_radius * probe_radius;

    // Calculate vector from p0 to p1
    let dp = p1 - p0;
    if ulps_eq!(dp.x(), T::Scalar::ZERO) && ulps_eq!(dp.y(), T::Scalar::ZERO) {
        // when the line is almost vertical we only use p1
        if (center - p1.to_2d()).magnitude_sq() <= r_sq {
            mt.insert(SearchResult::<T>::new(site_index, p1.z()));
        }
        return;
    }

    let dp_xy = dp.to_2d();
    let dep_xy_mag_sq = dp_xy.magnitude_sq();

    let t: T::Scalar = (center - p0.to_2d()).dot(dp.to_2d()) / dep_xy_mag_sq;
    let clamped_t = t.clamp(T::Scalar::ZERO, T::Scalar::ONE);

    // closest_point_xy is a point on the infinite line
    let clamped_closest_point_xy = p0.to_2d() + dp.to_2d() * clamped_t;
    let clamped_dist_sq_closest_point_xy = (center - clamped_closest_point_xy).magnitude_sq();
    if clamped_dist_sq_closest_point_xy > r_sq {
        return;
    }

    // closest_point_xy is a point on the infinite line
    let unclamped_closest_point_xy = p0.to_2d() + dp.to_2d() * t;
    let delta_d_sq = r_sq - (center - unclamped_closest_point_xy).magnitude_sq();
    if delta_d_sq < T::Scalar::ZERO {
        return;
    }
    // we know that dp.z >= 0 so we are skipping the .abs() for dp.z
    let m_sq = if dp.z() > T::Scalar::EPSILON {
        dp.z().powi(2) / (dp.x().powi(2) + dp.y().powi(2))
    } else {
        T::Scalar::ZERO
    };

    let clamped_closest_point = p0 + dp * t;
    let cz = clamped_closest_point.z() + (m_sq * delta_d_sq).sqrt();

    if cz > p1.z() {
        mt.insert(SearchResult::<T>::new(site_index, p1.z()));
        return;
    }
    if cz < p0.z() {
        mt.insert(SearchResult::new(site_index, p0.z()));
        return;
    }
    mt.insert(SearchResult::new(site_index, cz));
}

pub(super) fn shared_square_end_precompute_logic<T: GenericVector3, MESH: HasXYZ>(
    vertices: &[MESH],
    triangles: &[usize],
    probe_radius: T::Scalar,
) -> Vec<TriangleMetadata<T, MESH>>
where
    MESH: ConvertTo<T>,
{
    triangles
        .as_ref()
        .chunks(3)
        .map(|triangle| {
            TriangleMetadata::new_for_square_end(
                probe_radius,
                vertices[triangle[0]].to(),
                vertices[triangle[1]].to(),
                vertices[triangle[2]].to(),
            )
        })
        .collect()
}

// this function must use the CollisionFn signature
pub(super) fn square_end_compute_collision<T: GenericVector3, MESH: HasXYZ + ConvertTo<T>>(
    qp: &QueryParameters<'_, T, MESH>,
    site_index: usize,
    center: T::Vector2,
    mt: &mut MaximumTracker<SearchResult<T>>,
) where
    T::Scalar: approx::UlpsEq,
{
    let triangle_index = site_index * 3;
    let TriangleMetadata { delta_z, plane, .. } = &qp.meta_data[site_index];

    if let Some(PlaneMetadata {
        pft,
        translated_triangle: [p0, p1, p2],
        ..
    }) = plane
    {
        if is_inside_2d_triangle::<T::Vector2>(center, *p0, *p1, *p2) {
            // if the sphere center is inside the pre-computed triangle, we know that the plane detection
            // will result in the highest point, so there is no need to test the edges.
            mt.insert(SearchResult::new(
                site_index,
                pft.compute_z(center) + *delta_z,
            ));

            // no need to proceed with the edge detection
            return;
        }
    }
    //let center:DVec2 = center.into();

    let p0: T = qp.vertices[qp.indices[triangle_index]].to();
    let p1: T = qp.vertices[qp.indices[triangle_index + 1]].to();
    let p2: T = qp.vertices[qp.indices[triangle_index + 2]].to();

    square_end_to_edge_collision::<T>(center, qp.probe_radius, site_index, p0, p1, mt);
    square_end_to_edge_collision::<T>(center, qp.probe_radius, site_index, p1, p2, mt);
    square_end_to_edge_collision::<T>(center, qp.probe_radius, site_index, p2, p0, mt);
}

/// Calculates the Z-coordinate where a sphere intersects with an edge.The sphere can be considered to
/// fall from infinite Z, so the effect is the same as for a Ball Nose end mill.
///
/// This function determines the Z-coordinate at which a sphere, defined by its `center` and `probe_radius`,
/// intersects with a line segment defined by points `p0` and `p1`. Unlike other variants, this function
/// does not rely on transforming the sphere's location and the edge into a canonical form but rather works
/// directly in the provided coordinate space.
///
/// # Arguments
///
/// * `center`: The XY-coordinates of the center of the sphere.
/// * `probe_radius`: The radius of the sphere.
/// * `p0`: One end of the line segment. The function internally re-arranges `p0` and `p1` based on their Z-coordinates,
///         so their order in the argument list is not significant.
/// * `p1`: The other end of the line segment.
/// * `skip`: An enum value indicating whether to skip calculations related to one of the endpoints of the line segment.
///
/// # Returns
///
/// * An `Option<f64>` representing the highest Z-coordinate of intersection. If there is no intersection, it returns `None`.
///
/// # Notes
///
/// * The function may internally swap `p0` and `p1` to ensure `p0` always has a Z-coordinate less than or equal to `p1`.
///   This means callers do not need to pre-sort these points before calling the function.
/// * The `skip` argument allows for efficient handling of cases where one of the endpoints' calculations can be avoided.
///
/// # Examples
///
/// ```rust,ignore
/// let center = Vector2::new(1.0, 2.0);
/// let probe_radius = 1.5;
/// let p0 = Vector3::new(0.0, 0.0, 0.0);
/// let p1 = Vector3::new(2.0, 2.0, 2.0);
/// let z_value = ball_nose_to_edge_collision(center, probe_radius, p0, p1, SkipEndpoint::NoSkip);
/// ```
pub fn ball_nose_to_edge_collision<T: GenericVector3>(
    center: T::Vector2,
    probe_radius: T::Scalar,
    site_index: usize,
    mut p0: T,
    mut p1: T,
    mut skip: SkipEndpoint,
    mt: &mut MaximumTracker<SearchResult<T>>,
) {
    if p0.z() > p1.z() {
        // p1.z is now always higher or equal to p1.z
        std::mem::swap(&mut p0, &mut p1);
        skip = skip.flip();
    }

    let r_sq = probe_radius * probe_radius;

    // Calculate vector from p0 to p1
    let dp = p1 - p0;
    if dp.x().abs() < T::Scalar::EPSILON && dp.y().abs() < T::Scalar::EPSILON {
        // when the line is almost vertical we only use p1
        if skip != SkipEndpoint::SkipP1 {
            z_sphere_distance_to_single_point::<T>(
                site_index,
                p1,
                r_sq,
                (center - p1.to_2d()).magnitude_sq(),
                mt,
            );
        };
        return;
    }

    let dp_mag_sq = dp.magnitude_sq();
    let dp_xy = dp.to_2d();
    let dep_xy_mag_sq = dp_xy.magnitude_sq();

    let t = (center - p0.to_2d()).dot(dp.to_2d()) / dep_xy_mag_sq;

    // closest_point_xy is a point on the infinite line
    let closest_point_xy = p0.to_2d() + dp.to_2d() * t;
    let dist_sq_closest_point_xy = (center - closest_point_xy).magnitude_sq();

    // radius_factor_sq is a factor that diminishes by the radial distance from the edge.
    // at probe_radius the factor is 0.0 and 1.0 directly on top of the edge.
    let radius_factor_sq = T::Scalar::ONE - dist_sq_closest_point_xy / r_sq;
    if radius_factor_sq.is_sign_negative() {
        // closest point was out of reach of the radius
        return;
    }

    // we know that dp.z >= 0 so we are skipping the .abs() for dp.z
    let (m_factor_sq, t_offset) = if dp.z() > T::Scalar::EPSILON {
        let m_sq = dp.z().powi(2) / (dp.x().powi(2) + dp.y().powi(2));
        let m_factor_sq = T::Scalar::ONE + m_sq;
        // if point r is directly under the sphere, and point q is the
        // point that is actually touching the edge, d_rq_sq is the
        // distance between them squared.
        let d_rq_sq = r_sq * (m_sq.powi(2) + m_sq) / (m_sq + T::Scalar::ONE);
        (
            m_factor_sq,
            -(radius_factor_sq * d_rq_sq / dp_mag_sq).sqrt(),
        )
    } else {
        (T::Scalar::ONE, T::Scalar::ZERO)
    };

    if t > T::Scalar::ONE + t_offset {
        if skip != SkipEndpoint::SkipP1 {
            z_sphere_distance_to_single_point::<T>(
                site_index,
                p1,
                r_sq,
                (center - p1.to_2d()).magnitude_sq(),
                mt,
            )
        }
        return;
    }
    if t < t_offset {
        if skip != SkipEndpoint::SkipP0 {
            z_sphere_distance_to_single_point::<T>(
                site_index,
                p0,
                r_sq,
                (center - p0.to_2d()).magnitude_sq(),
                mt,
            );
        }
        return;
    }
    mt.insert(SearchResult::new(
        site_index,
        p0.z() + t * dp.z() + probe_radius * (m_factor_sq * radius_factor_sq).sqrt(),
    ))
}

pub(super) fn shared_ball_nose_precompute_logic<T: GenericVector3, MESH: HasXYZ + ConvertTo<T>>(
    vertices: &[MESH],
    indices: &[usize],
    probe_radius: T::Scalar,
) -> Vec<TriangleMetadata<T, MESH>> {
    indices
        .as_ref()
        .chunks(3)
        .map(|triangle| {
            TriangleMetadata::<T, MESH>::new_for_ball_nose(
                probe_radius,
                vertices[triangle[0]],
                vertices[triangle[1]],
                vertices[triangle[2]],
            )
        })
        .collect()
}

// this function must use the CollisionFn signature
pub(super) fn ball_nose_compute_collision<T: GenericVector3, MESH: HasXYZ + ConvertTo<T>>(
    qp: &QueryParameters<'_, T, MESH>,
    site_index: usize,
    center: T::Vector2,
    mt: &mut MaximumTracker<SearchResult<T>>,
) {
    let triangle_index = site_index * 3;
    let TriangleMetadata { delta_z, plane, .. } = &qp.meta_data[site_index];

    if let Some(PlaneMetadata {
        pft,
        translated_triangle: [p0, p1, p2],
        ..
    }) = plane
    {
        if is_inside_2d_triangle(center, *p0, *p1, *p2) {
            // if the sphere center is inside the pre-computed triangle, we know that the plane detection
            // will result in the highest point, so there is no need to test the edges.
            mt.insert(SearchResult::new(
                site_index,
                pft.compute_z(center) + *delta_z,
            ));

            // no need to proceed with the edge detection
            return;
        }
    }
    //let probe_radius = T::Scalar::to_f64(qp.probe_radius);
    let p0 = qp.vertices[qp.indices[triangle_index]].to();
    let p1 = qp.vertices[qp.indices[triangle_index + 1]].to();
    let p2 = qp.vertices[qp.indices[triangle_index + 2]].to();

    ball_nose_to_edge_collision::<T>(
        center,
        qp.probe_radius,
        site_index,
        p0,
        p1,
        SkipEndpoint::SkipP1,
        mt,
    );
    ball_nose_to_edge_collision::<T>(
        center,
        qp.probe_radius,
        site_index,
        p1,
        p2,
        SkipEndpoint::SkipP1,
        mt,
    );
    ball_nose_to_edge_collision::<T>(
        center,
        qp.probe_radius,
        site_index,
        p2,
        p0,
        SkipEndpoint::SkipP1,
        mt,
    );
}
/*
/// Computes the Z distance from a single point to the origin in a transformed space.
///
/// This function is designed to work in a specific coordinate space where the sample point
/// is located at the origin. The function calculates the Z distance from the given point `p`
/// to the origin, considering the squared radius `r_sq`.
///
/// The coordinate space must be previously transformed (translated and rotated) such that
/// the sample point (or reference point) is at the origin. In this transformed space, the
/// function calculates the Z offset based on the XY distance from `p` to the origin and
/// the provided squared radius.
///
/// # Arguments
///
/// * `p` - The 3D point in the transformed space.
/// * `r_sq` - The squared radius to be considered for the distance calculation.
///
/// # Returns
///
/// * `Some(f64)` - The computed Z distance if the point `p` is within the radius.
/// * `None` - If the point `p` is outside the radius.
///
/// # Notes
///
/// Ensure that this function is only used with coordinates in the appropriate transformed space.
#[allow(dead_code)]
#[inline]
fn z_distance_to_single_point_transformed(p: Vector3, r_sq: f64) -> Option<f64> {
    let dist_sq_diff = r_sq - p.xy().magnitude_sq();
    if dist_sq_diff.is_sign_negative() {
        // the point was outside the range of the radius
        None
    } else {
        Some(p.z + dist_sq_diff.sqrt())
    }
}*/

#[inline(always)]
fn z_sphere_distance_to_single_point<T: GenericVector3>(
    site_index: usize,
    p: T,
    r_sq: T::Scalar,
    dist_sq: T::Scalar,
    mt: &mut MaximumTracker<SearchResult<T>>,
) {
    let dist_sq_diff = r_sq - dist_sq;
    if dist_sq_diff.is_sign_positive() {
        mt.insert(SearchResult::new(site_index, p.z() + dist_sq_diff.sqrt()));
    }
}

/*
/// Calculates the Z-coordinate where a sphere intersects with an edge.
///
/// This function determines the Z-coordinate at which a sphere, defined by its `center` and `probe_radius`,
/// intersects with a line segment defined by points `p0` and `p1`. This function rely on transforming
/// the sphere's location and the edge into a canonical form. It might be a little bit slower than the non-transforming function.
///
/// # Arguments
///
/// * `center`: The XY-coordinates of the center of the sphere.
/// * `probe_radius`: The radius of the sphere.
/// * `p0`: One end of the line segment. The function internally re-arranges `p0` and `p1` based on their Z-coordinates,
///         so their order in the argument list is not significant.
/// * `p1`: The other end of the line segment.
/// * `skip`: An enum value indicating whether to skip calculations related to one of the endpoints of the line segment.
///
/// # Returns
///
/// * An `Option<f64>` representing the Z-coordinate of intersection. If there is no intersection, it returns `None`.
///
/// # Notes
///
/// * The function may internally swap `p0` and `p1` to ensure `p0` always has a Z-coordinate less than or equal to `p1`.
///   This means callers do not need to pre-sort these points before calling the function.
/// * The `skip` argument allows for efficient handling of cases where one of the endpoints' calculations can be avoided.
///
/// # Examples
///
/// ```rust,ignore
/// let center = Vector2::new(1.0, 2.0);
/// let probe_radius = 1.5;
/// let p0 = Vector3::new(0.0, 0.0, 0.0);
/// let p1 = Vector3::new(2.0, 2.0, 2.0);
/// let z_value = z_coord_sphere_to_edge_1(center, probe_radius, p0, p1, SkipEndpoint::NoSkip);
/// ```
pub fn z_coord_sphere_to_edge_1(
    center: Vector2,
    probe_radius: f64,
    mut p0: DVec3,
    mut p1: DVec3,
    mut skip: SkipEndpoint,
) -> Option<f64> {
    if p0.z > p1.z {
        // p1.z is now always higher or equal to p0.z
        std::mem::swap(&mut p0, &mut p1);
        skip = skip.flip();
    }
    let r_sq = probe_radius * probe_radius;

    // use translation and rotation to place center at origin and align p0-p1 with the x-axis
    let transform = RigidTransform2D::translate_rotate_align_x(center, p0.xy(), p1.xy());
    p0 = transform.transform_point(p0);
    p1 = transform.transform_point(p1);

    // radius_factor_sq is a factor that diminishes by the radial distance from the edge.
    // at probe_radius the factor is 0.0 and 1.0 directly on top of the edge.
    // the closest point to the infinite line is simply p0.y.abs()
    let radius_factor_sq = 1.0 - p0.y.powi(2) / r_sq;
    if radius_factor_sq.is_sign_negative() {
        // closest point was out of reach of the radius
        return None;
    }

    // Calculate vector from p0 to p1
    let dp = p1 - p0;

    if dp.x.abs() < EPSILON {
        // when the line is almost vertical we only use p1
        return if skip != SkipEndpoint::SkipP1 {
            z_distance_to_single_point_transformed(p1, r_sq)
        } else {
            None
        };
    }

    // we know that unit_dp.z >= 0 so we are skipping the .abs() for dp.z
    let (m_factor_sq, t_offset) = if dp.z > EPSILON {
        let dp_mag_sq = dp.x.powi(2) + dp.z.powi(2);
        let m_sq = (dp.z / dp.x).powi(2);
        let m_factor_sq = 1.0 + m_sq;
        // if point r is directly under the sphere, and point q is the
        // point that is actually touching the edge, d_rq_sq is the
        // distance between them squared.
        let d_rq_sq = r_sq * (m_sq.powi(2) + m_sq) / (m_sq + 1.0);
        (
            m_factor_sq,
            -(radius_factor_sq * d_rq_sq / dp_mag_sq).sqrt(),
        )
    } else {
        (1.0, 0.0)
    };

    let t = p0.x / (p0.x - p1.x);
    if t > 1.0 + t_offset {
        return if skip != SkipEndpoint::SkipP1 {
            z_distance_to_single_point_transformed(p1, r_sq)
        } else {
            None
        };
    }
    if t < t_offset {
        return if skip != SkipEndpoint::SkipP0 {
            z_distance_to_single_point_transformed(p0, r_sq)
        } else {
            None
        };
    }
    Some(p0.z + t * dp.z + probe_radius * (m_factor_sq * radius_factor_sq).sqrt())
}

// This one works, don't touch!
pub fn z_coord_sphere_to_edge_2(
    center: Vector2,
    probe_radius: f64,
    p0: Vector3,
    p1: Vector3,
) -> Option<f64> {
    let r = probe_radius;
    let x = center.x;
    let y = center.y;
    let (x0, y0, z0) = (p0.x, p0.y, p0.z);
    let (x1, y1, z1) = (p1.x, p1.y, p1.z);

    // generated by sympy
    let x2 = z0.powi(2);
    let x3 = z0 * z1;
    let x4 = x1.powi(2);
    let x5 = y1.powi(2);
    let x6 = x0 * x1;
    let x7 = 2.0 * x6;
    let x8 = y0 * y1;
    let x9 = 2.0 * x8;
    let x10 = x0.powi(2);
    let x11 = y0.powi(2);
    let x12 = x10 + x11;
    let x13 = x12 + x4 + x5 - x7 - x9;
    let x14 = x13 + x2 - 2.0 * x3 + z1.powi(2);
    let x15 = x * x1;
    let x16 = y * y1;
    let x17 = 1.0 / x13;
    let x18 = x * x0;
    let x19 = y * y0;
    let x20 = 2.0 * x18;
    let x21 = 2.0 * x15;
    let x22 = x.powi(2);
    let x23 = y.powi(2);
    let x24 = r.powi(2);

    let t = (x12 + x15 + x16 - x18 - x19 + x2 - x3 - x6 - x8
        + (-z0 + z1)
            * (x17
                * f64::sqrt(
                    -x14 * (-2.0 * x10 * x16 + x10 * x23 - x10 * x24 + x10 * x5 - x11 * x21
                        + x11 * x22
                        - x11 * x24
                        + x11 * x4
                        + x15 * x9
                        + x16 * x20
                        - x16 * x21
                        + x16 * x7
                        + x18 * x9
                        - x19 * x20
                        + x19 * x21
                        - 2.0 * x19 * x4
                        + x19 * x7
                        - x20 * x5
                        + x22 * x5
                        - x22 * x9
                        + x23 * x4
                        - x23 * x7
                        - x24 * x4
                        - x24 * x5
                        + x24 * x7
                        + x24 * x9
                        - x7 * x8),
                )
                + x17
                    * (x10 * z1 + x11 * z1 - x15 * z0 + x15 * z1 - x16 * z0
                        + x16 * z1
                        + x18 * z0
                        - x18 * z1
                        + x19 * z0
                        - x19 * z1
                        + x4 * z0
                        + x5 * z0
                        - x6 * z0
                        - x6 * z1
                        - x8 * z0
                        - x8 * z1)))
        / x14;

    if !t.is_finite() {
        //println!("z_coord_sphere_to_edge_3 would have returned a non-finite z t:{}", t);
        //println!("center:{:?},r:{:?}, p0:{:?}, p1:{:?}", center,probe_radius, p0, p1);
        return None;
    }
    if t > 0.0 && t < 1.0 {
        // generated by sympy
        let x2 = x0.powi(2);
        let x3 = x1.powi(2);
        let x4 = y0.powi(2);
        let x5 = y1.powi(2);
        let x6 = x0 * x1;
        let x7 = 2.0 * x6;
        let x8 = y0 * y1;
        let x9 = 2.0 * x8;
        let x10 = x2 + x3 + x4 + x5 - x7 - x9;
        /*if !x10.is_normal() {
            println!("z_coord_sphere_to_edge_3 would have returned a non-finite z x10:{}", x10);
            println!("center:{:?},r:{:?}, p0:{:?}, p1:{:?}", center,probe_radius, p0, p1);
            return None
        }*/
        let x11 = 1.0 / x10;
        let x12 = x * x0;
        let x13 = x * x1;
        let x14 = y * y0;
        let x15 = y * y1;
        let x16 = 2.0 * x12;
        let x17 = 2.0 * x13;
        let x18 = x.powi(2);
        let x19 = y.powi(2);
        let x20 = r.powi(2);

        let sz =
            x11 * f64::sqrt(
                (-x10 - z0.powi(2) + 2.0 * z0 * z1 - z1.powi(2))
                    * (x12 * x9 + x13 * x9 - x14 * x16 + x14 * x17 - 2.0 * x14 * x3
                        + x14 * x7
                        + x15 * x16
                        - x15 * x17
                        - 2.0 * x15 * x2
                        + x15 * x7
                        - x16 * x5
                        - x17 * x4
                        + x18 * x4
                        + x18 * x5
                        - x18 * x9
                        + x19 * x2
                        + x19 * x3
                        - x19 * x7
                        - x2 * x20
                        + x2 * x5
                        - x20 * x3
                        - x20 * x4
                        - x20 * x5
                        + x20 * x7
                        + x20 * x9
                        + x3 * x4
                        - x7 * x8),
            ) + x11
                * (x12 * z0 - x12 * z1 - x13 * z0 + x13 * z1 + x14 * z0 - x14 * z1 - x15 * z0
                    + x15 * z1
                    + x2 * z1
                    + x3 * z0
                    + x4 * z1
                    + x5 * z0
                    - x6 * z0
                    - x6 * z1
                    - x8 * z0
                    - x8 * z1);
        if !sz.is_finite() {
            //println!("z_coord_sphere_to_edge_3 would have returned a non-finite z {}", sz);
            //println!("center:{:?},r:{:?}, p0:{:?}, p1:{:?}", center,probe_radius, p0, p1);
            None
        } else {
            Some(sz)
        }
    } else {
        if t > 1.0 {
            // hope for the next endpoint to cover this
            return None;
        }
        let s_r_squared = r * r;

        let (pz, dxy_squared) = if t < 0.0 {
            (p0.z, (center - p0.xy()).magnitude_sq())
        } else {
            (p1.z, (center - p1.xy()).magnitude_sq())
        };

        if dxy_squared > s_r_squared {
            return None;
        }

        let delta_z = (s_r_squared - dxy_squared).sqrt();
        //assert!((pz + delta_z).is_finite());
        Some(pz + delta_z)
    }
}

// This works, don't mess with it
pub fn z_coord_sphere_to_edge_3(
    center: Vector2,
    probe_radius: f64,
    p0: Vector3,
    p1: Vector3,
) -> Option<f64> {
    let (r, r_sq) = (probe_radius, probe_radius * probe_radius);
    if distance_sq_point_to_segment(center, p0.xy(), p1.xy()) > r_sq {
        return None;
    }

    let x = center.x;
    let y = center.y;
    let (x0, y0, z0) = (p0.x, p0.y, p0.z);
    let (x1, y1, z1) = (p1.x, p1.y, p1.z);

    // generated by sympy
    let x2 = z0.powi(2);
    let x3 = z0 * z1;
    let x4 = x1.powi(2);
    let x5 = y1.powi(2);
    let x6 = x0 * x1;
    let x7 = 2.0 * x6;
    let x8 = y0 * y1;
    let x9 = 2.0 * x8;
    let x10 = x0.powi(2);
    let x11 = y0.powi(2);
    let x12 = x10 + x11;
    let x13 = x12 + x4 + x5 - x7 - x9;
    let x14 = x13 + x2 - 2.0 * x3 + z1.powi(2);
    let x15 = x * x1;
    let x16 = y * y1;
    let x17 = 1.0 / x13;
    let x18 = x * x0;
    let x19 = y * y0;
    let x20 = 2.0 * x18;
    let x21 = 2.0 * x15;
    let x22 = x.powi(2);
    let x23 = y.powi(2);
    let x24 = r.powi(2);

    let t = (x12 + x15 + x16 - x18 - x19 + x2 - x3 - x6 - x8
        + (-z0 + z1)
            * (x17
                * f64::sqrt(
                    -x14 * (-2.0 * x10 * x16 + x10 * x23 - x10 * x24 + x10 * x5 - x11 * x21
                        + x11 * x22
                        - x11 * x24
                        + x11 * x4
                        + x15 * x9
                        + x16 * x20
                        - x16 * x21
                        + x16 * x7
                        + x18 * x9
                        - x19 * x20
                        + x19 * x21
                        - 2.0 * x19 * x4
                        + x19 * x7
                        - x20 * x5
                        + x22 * x5
                        - x22 * x9
                        + x23 * x4
                        - x23 * x7
                        - x24 * x4
                        - x24 * x5
                        + x24 * x7
                        + x24 * x9
                        - x7 * x8),
                )
                + x17
                    * (x10 * z1 + x11 * z1 - x15 * z0 + x15 * z1 - x16 * z0
                        + x16 * z1
                        + x18 * z0
                        - x18 * z1
                        + x19 * z0
                        - x19 * z1
                        + x4 * z0
                        + x5 * z0
                        - x6 * z0
                        - x6 * z1
                        - x8 * z0
                        - x8 * z1)))
        / x14;

    // we will handle t>1.0 in the next edge of the triangle
    if !t.is_finite() {
        //|| t > 1.0 {
        return None;
    }
    if (0.0..=1.0).contains(&t) {
        let p0p1 = p1 - p0;
        let q = p0 + t * p0p1;
        let sz = q.z + ((q.x - x).powi(2) + (q.y - y).powi(2) - r_sq).abs().sqrt();
        if !sz.is_finite() {
            return None;
        }
        Some(sz)
    } else if t < 0.0 {
        let dxy_sq = (center - p0.xy()).magnitude_sq();
        if dxy_sq > r_sq {
            return None;
        }
        Some(p0.z + (r_sq - dxy_sq).sqrt())
    } else
    /*if t > 1.0 */
    {
        let dxy_sq = (center - p1.xy()).magnitude_sq();
        if dxy_sq > r_sq {
            return None;
        }
        Some(p1.z + (r_sq - dxy_sq).sqrt())
    }
}

pub fn z_coord_sphere_to_edge_4(
    center: Vector2,
    probe_radius: f64,
    p0: Vector3,
    p1: Vector3,
) -> Option<f64> {
    let r = probe_radius;
    let r_sq = r * r;

    let (x0, y0, z0) = (p0.x, p0.y, p0.z);
    let (x0_sq, y0_sq, z0_sq) = (x0 * x0, y0 * y0, z0 * z0);
    let (x1, y1, z1) = (p1.x, p1.y, p1.z);
    let (x1_sq, y1_sq, z1_sq) = (x1 * x1, y1 * y1, z1 * z1);
    let (sx, sy) = (center.x, center.y);
    let (sx_sq, sy_sq) = (sx * sx, sy * sy);

    let x0_x1 = x0 * x1;
    let y0_y1 = y0 * y1;
    let z0_z1 = z0 * z1;
    let x0_y0 = x0 * y0;
    let x0_y1 = x0 * y1;
    let x1_y0 = x1 * y0;
    let x1_y1 = x1 * y1;
    let sx_sy = sx * sy;

    let denom = x0.powi(2) - 2.0 * x0_x1 + x1_sq + y0_sq - 2.0 * y0_y1 + y1_sq;
    if denom == 0.0 {
        return None;
    }

    // generated by sympy
    let z = (sx * x0 * z0 - sx * x0 * z1 - sx * x1 * z0 + sx * x1 * z1 + sy * y0 * z0
        - sy * y0 * z1
        - sy * y1 * z0
        + sy * y1 * z1
        + x0_sq * z1
        - x0_x1 * z0
        - x0_x1 * z1
        + x1_sq * z0
        + y0_sq * z1
        - y0_y1 * z0
        - y0_y1 * z1
        + y1_sq * z0
        + f64::sqrt(
            -(x0_sq - 2.0 * x0_x1 + x1_sq + y0_sq - 2.0 * y0 * y1 + y1_sq + z0_sq - 2.0 * z0_z1
                + z1_sq)
                * (sx_sq * y0_sq - 2.0 * sx_sq * y0 * y1 + sx_sq * y1_sq - 2.0 * sx_sy * x0_y0
                    + 2.0 * sx_sy * x0_y1
                    + 2.0 * sx_sy * x1_y0
                    - 2.0 * sx_sy * x1_y1
                    + 2.0 * sx * x0 * y0_y1
                    - 2.0 * sx * x0 * y1_sq
                    - 2.0 * sx * x1 * y0_sq
                    + 2.0 * sx * x1 * y0_y1
                    + sy_sq * x0_sq
                    - 2.0 * sy_sq * x0_x1
                    + sy_sq * x1_sq
                    - 2.0 * sy * x0_sq * y1
                    + 2.0 * sy * x0_x1 * y0
                    + 2.0 * sy * x0_x1 * y1
                    - 2.0 * sy * x1_sq * y0
                    - r_sq * x0_sq
                    + 2.0 * r_sq * x0 * x1
                    - r_sq * x1_sq
                    - r_sq * y0_sq
                    + 2.0 * r_sq * y0 * y1
                    - r_sq * y1_sq
                    + x0_sq * y1_sq
                    - 2.0 * x0 * x1 * y0 * y1
                    + x1_sq * y0_sq),
        ))
        / denom;
    if !z.is_finite() {
        return None;
    }

    // i wish i had solved for t instead, then i would not have to do this:
    let s = center.xyz(z);
    let dp_xyz = p1 - p0;
    let t: f64 = (s - p0).dot(&dp_xyz) / dp_xyz.magnitude_sq();
    if t <= 0.0 {
        // calculate using the endpoint p0
        let dxy_squared = (center - p0.xy()).magnitude_sq();
        if dxy_squared > r_sq {
            return None;
        }
        return Some(p0.z + (r_sq - dxy_squared).sqrt());
    } else if t >= 1.0 {
        // calculate using the endpoint p1
        let dxy_squared = (center - p1.xy()).magnitude_sq();
        if dxy_squared > r_sq {
            return None;
        }
        return Some(p1.z + (r_sq - dxy_squared).sqrt());
    }
    Some(z)
}

pub fn z_coord_sphere_to_edge_5(
    center: Vector2,
    probe_radius: f64,
    p0: Vector3,
    p1: Vector3,
) -> Option<f64> {
    let r_sq = probe_radius * probe_radius;
    if distance_sq_point_to_segment(center, p0.xy(), p1.xy()) > r_sq {
        return None;
    }

    let (sx, sy) = (center.x, center.y);
    let (sx_sq, sy_sq) = (sx * sx, sy * sy);
    let (x0, y0, z0) = (p0.x, p0.y, p0.z);
    let (x0_sq, y0_sq) = (x0 * x0, y0 * y0);
    let Vector3 {
        x: dx,
        y: dy,
        z: dz,
    } = p1 - p0;
    let (dx_sq, dy_sq, dz_sq) = (dx * dx, dy * dy, dz * dz);

    // generated by sympy
    let factor = dx.powi(4) + 2.0 * dx_sq * dy_sq - dx_sq * dz_sq + dy.powi(4) - dy_sq * dz_sq;
    if !factor.is_finite() {
        return None;
    }
    let t = if dz >= 0.0 {
        (dz * (-(dx_sq + dy_sq - dz_sq)
            * (dx_sq * r_sq - dx_sq * sy_sq + 2.0 * dx_sq * sy * y0 - dx_sq * y0_sq
                + 2.0 * dx * dy * sx * sy
                - 2.0 * dx * dy * sx * y0
                - 2.0 * dx * dy * sy * x0
                + 2.0 * dx * dy * x0 * y0
                + dy_sq * r_sq
                - dy_sq * sx_sq
                + 2.0 * dy_sq * sx * x0
                - dy_sq * x0_sq))
            .abs()
            .sqrt()
            * (dx_sq + dy_sq)
            + (dx * sx - dx * x0 + dy * sy - dy * y0) * factor)
            / ((dx_sq + dy_sq) * factor)
    } else {
        (-dz * (-(dx_sq + dy_sq - dz_sq)
            * (dx_sq * r_sq - dx_sq * sy_sq + 2.0 * dx_sq * sy * y0 - dx_sq * y0_sq
                + 2.0 * dx * dy * sx * sy
                - 2.0 * dx * dy * sx * y0
                - 2.0 * dx * dy * sy * x0
                + 2.0 * dx * dy * x0 * y0
                + dy_sq * r_sq
                - dy_sq * sx_sq
                + 2.0 * dy_sq * sx * x0
                - dy_sq * x0_sq))
            .abs()
            .sqrt()
            * (dx_sq + dy_sq)
            + (dx * sx - dx * x0 + dy * sy - dy * y0) * factor)
            / ((dx_sq + dy_sq) * factor)
    };

    // we will handle t>1.0 in the next edge of the triangle
    if !t.is_finite() {
        //t =  (-d_x*p0x + d_x*sx - d_y*p0y + d_y*sy + f64::sqrt(-d_x.powi(2)*p0y.powi(2) + 2.0*d_x.powi(2)*p0y*sy + d_x.powi(2)*r.powi(2) - d_x.powi(2)*sy.powi(2) + 2.0*d_x*d_y*p0x*p0y - 2.0*d_x*d_y*p0x*sy - 2.0*d_x*d_y*p0y*sx + 2.0*d_x*d_y*sx*sy - d_y.powi(2)*p0x.powi(2) + 2.0*d_y.powi(2)*p0x*sx + d_y.powi(2)*r.powi(2) - d_y.powi(2)*sx.powi(2)))/(d_x.powi(2) + d_y.powi(2));
        //if !t.is_finite() {
        return None;
        //}
    }
    if (0.0..=1.0).contains(&t) {
        /*let p0p1 = p1 - p0;
        let q = p0 + t * p0p1;
        let sz = q.z
            + ((q.x - sx).powi(2) + (q.y - sy).powi(2) - r_sq)
                .abs()
                .sqrt();
        */
        // why is the .abs() required?
        let sz = dz * t
            + z0
            + (-r_sq + (dx * t - sx + x0).powi(2) + (dy * t - sy + y0).powi(2))
                .abs()
                .sqrt();
        if !sz.is_finite() {
            return None;
        }
        Some(sz)
    } else if t < 0.0 {
        let dxy_sq = (center - p0.xy()).magnitude_sq();
        if dxy_sq > r_sq {
            return None;
        }
        Some(p0.z + (r_sq - dxy_sq).sqrt())
    } else
    /*if t > 1.0 */
    {
        let dxy_sq = (center - p1.xy()).magnitude_sq();
        if dxy_sq > r_sq {
            return None;
        }
        Some(p1.z + (r_sq - dxy_sq).sqrt())
    }
}
*/
