// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.
mod impls;
#[cfg(test)]
mod tests;

use std::{
    fmt,
    fmt::{Debug, Display},
    ops::Sub,
};
use vector_traits::{
    glam::{DVec2, DVec3, Vec2, Vec3, Vec3A},
    num_traits::Float,
    GenericScalar, GenericVector2, GenericVector3, HasXY, HasXYZ,
};

#[derive(PartialEq, Copy, Clone)]
pub struct Circle<T: GenericVector2> {
    pub center: T,
    pub radius: T::Scalar,
}

impl<T: GenericVector2> Circle<T> {
    pub fn new(center: T, radius: T::Scalar) -> Self {
        Self { center, radius }
    }
}

pub fn centroid_2d<T: GenericVector2>(p0: T, p1: T, p2: T) -> T {
    let x = (p0.x() + p1.x() + p2.x()) / T::Scalar::THREE;
    let y = (p0.y() + p1.y() + p2.y()) / T::Scalar::THREE;
    T::new_2d(x, y)
}

/// Represents the area of a 2D triangle.
///
/// Although the term "area" is commonly understood to refer to a 2D space,
/// it's essential to specify that `Area2D` represents the area of a triangle
/// defined within a 2D space, rather than being derived from the projection
/// of a 3D triangle. Typically this area are created by the area_2d() function.
///
/// This distinction helps avoid potential confusion when working with both
/// 2D and 3D geometries in the same codebase.
#[derive(PartialEq, PartialOrd, Debug, Clone, Copy)]
pub struct Area2D<T: GenericVector2>(T::Scalar);

impl<T: GenericVector2> Area2D<T> {
    /// Returns the underlying value of the 2D area.
    #[inline]
    pub fn value(&self) -> T::Scalar {
        self.0
    }

    /// Calculates the area of a 2D triangle defined by three vertices.
    ///
    /// # Parameters
    ///
    /// - `p0`: The first vertex of the triangle.
    /// - `p1`: The second vertex of the triangle.
    /// - `p2`: The third vertex of the triangle.
    ///
    /// # Returns
    ///
    /// An `Area2D` value representing the area of the triangle.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let p0 = Vector2::new(0.0, 0.0);
    /// let p1 = Vector2::new(1.0, 0.0);
    /// let p2 = Vector2::new(0.0, 1.0);
    ///
    /// let area = Area2d::new(p0, p1, p2);
    /// assert_eq!(area.value(), 0.5);
    /// ```
    #[inline]
    pub fn new(p0: T, p1: T, p2: T) -> Self {
        Area2D(
            (p0.x() * (p1.y() - p2.y()) + p1.x() * (p2.y() - p0.y()) + p2.x() * (p0.y() - p1.y()))
                .abs()
                / T::Scalar::TWO,
        )
    }
}

/// Represents the area of a 3D triangle.
///
/// Although the term "area" is commonly understood to refer to a 2D space,
/// it's essential to specify that `Area3D` represents the area of a triangle
/// defined within a 3D space, rather than being derived from the projection
/// of a 3D triangle. Typically this area are created by the area_3d() function.
///
/// This distinction helps avoid potential confusion when working with both
/// 2D and 3D geometries in the same codebase.

/// Represents a plane in 3D space defined by a triangle.
///
/// Given a triangle defined by three 3D points, this structure provides
/// a way to compute the Z-coordinate of any point `(x, y)` on the plane
/// of the triangle.
///
/// # Examples
///
/// ```rust,ignore
/// let p0 = Vector3::new(1.0, 0.0, 2.0);
/// let p1 = Vector3::new(0.0, 2.0, 3.0);
/// let p2 = Vector3::new(2.0, 2.0, 4.0);
///
/// let plane = PlaneFromTriangle::new(p0, p1, p2);
/// let z = plane.compute_z(&Vector2::new(1.0, 2.0));
/// ```
pub struct PlaneFromTriangle<T: GenericVector2> {
    a_over_c: T::Scalar,
    b_over_c: T::Scalar,
    d_over_c: T::Scalar,
}

impl<T: GenericVector2> PlaneFromTriangle<T> {
    /// Constructs a new `PlaneFromTriangle` from three non-collinear 3D points.
    ///
    /// # Arguments
    ///
    /// * `p0` - First point of the triangle.
    /// * `p1` - Second point of the triangle.
    /// * `p2` - Third point of the triangle.
    ///
    /// # Panics
    ///
    /// Panics if the three points are collinear.
    #[allow(dead_code)]
    pub fn new(p0: T::Vector3, p1: T::Vector3, p2: T::Vector3) -> Self {
        let n = (p1 - p0).cross(p2 - p0); // Normal of the plane
        let a = n.x();
        let b = n.y();
        let c = n.z();

        let d = -(a * p0.x() + b * p0.y() + c * p0.z());

        PlaneFromTriangle {
            a_over_c: -a / c,
            b_over_c: -b / c,
            d_over_c: -d / c,
        }
    }

    /// Constructs a new `PlaneFromTriangle` from a point and a normal.
    ///
    /// # Arguments
    ///
    /// * `normal` - The normal vector to the plane.
    /// * `p0` - A point on the plane.
    ///
    /// # Panics
    ///
    /// Panics if the normal is a zero vector.
    pub fn new_from_normal(normal: T::Vector3, p0: T::Vector3) -> Self {
        let a = normal.x();
        let b = normal.y();
        let c = normal.z();

        let d = -(a * p0.x() + b * p0.y() + c * p0.z());

        PlaneFromTriangle {
            a_over_c: -a / c,
            b_over_c: -b / c,
            d_over_c: -d / c,
        }
    }

    /// This constructor is used for a perfectly flat (in Z) plane
    /// It will only return a single coordinate
    #[allow(dead_code)]
    pub fn new_from_z_coord(z_coord: T::Scalar) -> Self {
        PlaneFromTriangle {
            a_over_c: T::Scalar::ZERO,
            b_over_c: T::Scalar::ZERO,
            d_over_c: z_coord,
        }
    }

    /// Computes the Z-coordinate for a given `(x, y)` on the plane of the triangle.
    ///
    /// # Arguments
    ///
    /// * `x` - X-coordinate of the point.
    /// * `y` - Y-coordinate of the point.
    ///
    /// # Returns
    ///
    /// Returns the Z-coordinate of the point `(x, y)` on the plane.
    pub fn compute_z(&self, v: T) -> T::Scalar {
        self.a_over_c * v.x() + self.b_over_c * v.y() + self.d_over_c
    }
}

/// A 2D rigid transformation structure optimized for translations and rotations.
///
/// This struct represents a subset of 2D affine transformations, specifically
/// those involving translations and rotations. It operates on `Vector3` types,
/// but it does not modify the Z coordinate in any transformation.
///
/// A "rigid transformation" is one that preserves distances between points,
/// meaning there are no scalings or shearing operations involved. Only translations
/// and rotations are supported.
///
/// The matrix form of this transformation is represented as:
///
/// ```plaintext
/// | m00  m01  m02 |
/// | m10  m11  m12 |
/// |  0    0    1  |
/// ```
///
/// The last row (0, 0, 1) is implicit and isn't stored in the struct, optimizing
/// for memory and performance.
///
///
/// Note: The `RigidTransform2D` struct is not suitable for general 3D transformations.
/// It is tailored specifically for 2D operations on `Vector3` where the Z coordinate
/// remains invariant.
pub struct RigidTransform2D<T>
where
    T: GenericVector3,
{
    m00: T::Scalar,
    m01: T::Scalar,
    m02: T::Scalar,

    m10: T::Scalar,
    m11: T::Scalar,
    m12: T::Scalar,
}

impl<T> RigidTransform2D<T>
where
    T: GenericVector3,
{
    pub fn identity() -> Self {
        RigidTransform2D {
            m00: T::Scalar::ONE,
            m01: T::Scalar::ZERO,
            m02: T::Scalar::ZERO,
            m10: T::Scalar::ZERO,
            m11: T::Scalar::ONE,
            m12: T::Scalar::ZERO,
        }
    }

    /// Creates a `RigidTransform2D` that translates the "world" origin to the specified `translation`
    /// point and rotates the given edge so it becomes parallel to the X axis.
    ///
    /// This transformation does not move `p0` to the new origin. Instead, it applies the necessary
    /// rotation and translation so that the segment formed by `p0` to `p1` aligns with the X axis,
    /// while the "world" origin is shifted to the `translation` point.
    ///
    /// # Arguments
    ///
    /// * `translation` - The new "world" origin after the transformation.
    /// * `p0` - The starting point of the edge.
    /// * `p1` - The ending point of the edge.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let p0 = Vector3::new(1.0, 2.0, 5.0);
    /// let p1 = Vector3::new(3.0, 2.0, 10.0);
    /// let transform = RigidTransform2D::translate_rotate_align_x(Vector2::new(5.0, 2.0), p0.xy(), p1.xy());
    /// let p0_transformed = transform.transform_point(p0);
    /// let p1_transformed = transform.transform_point(p1);
    /// assert_eq!(p0_transformed.y, p1_transformed.y);
    /// ```
    ///
    /// Note: This function operates in 2D space, and any transformation applied to a `Vector3`
    /// using the resulting `RigidTransform2D` will leave the Z coordinate unchanged.
    pub fn translate_rotate_align_x(
        translation: T::Vector2,
        p0: T::Vector2,
        p1: T::Vector2,
    ) -> Self {
        // Calculate the rotation components
        let d = (p1.sub(p0)).safe_normalize().unwrap();
        let cos_theta = d.x();
        let neg_sin_theta = -d.y();

        // Translate and then rotate the translation vector
        let rotated_tx = translation.x() * cos_theta - translation.y() * neg_sin_theta;
        let rotated_ty = translation.x() * neg_sin_theta + translation.y() * cos_theta;
        let mut matrix = RigidTransform2D::identity();
        matrix.m00 = cos_theta;
        matrix.m01 = -neg_sin_theta;
        matrix.m02 = -rotated_tx;

        matrix.m10 = neg_sin_theta;
        matrix.m11 = cos_theta;
        matrix.m12 = -rotated_ty;

        matrix
    }

    /// Transforms the given `Vector3` point using this `RigidTransform2D`.
    ///
    /// This function applies the transformation to the `Vector3` by performing a matrix multiplication.
    /// Specifically, it computes the resulting point as:
    ///
    /// ```rust,ignore
    /// Vector3(m00*x + m01*y + m02,
    ///         m10*x + m11*y + m12,
    ///         z)
    /// ```
    ///
    /// Where `mij` are the elements of this `RigidTransform2D` and `(x, y, z)` are the coordinates of the input `Vector3`.
    ///
    /// # Arguments
    ///
    /// * `p` - The `Vector3` point to be transformed.
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let transform = RigidTransform2D::new(/* ... */); // some constructor or initialization method
    /// let point = Vector3::new(1.0, 2.0, 3.0);
    /// let transformed_point = transform.transform_point(point);
    /// ```
    ///
    /// Note: This method only affects the X and Y coordinates of the `Vector3`. The Z coordinate remains unchanged.

    pub fn transform_point(&self, p: T) -> T {
        //println!("\nM:{:?}", self);
        //println!("p:{:?}", p);
        let x = self.m00 * p.x() + self.m01 * p.y() + self.m02;
        let y = self.m10 * p.x() + self.m11 * p.y() + self.m12;

        T::new_3d(x, y, p.z())
    }

    /// Computes the inverse of this `RigidTransform2D`.
    ///
    /// The inverse transformation will convert points transformed by this `RigidTransform2D` back to their original positions in world coordinates.
    ///
    /// # Returns
    ///
    /// * An `Option<RigidTransform2D>` containing the inverse transformation if it exists.
    /// * `None` if the inverse does not exist (e.g., if this transformation is singular and cannot be inverted).
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let transform = RigidTransform2D::new(/* ... */); // some constructor or initialization method
    /// let point = Vector3::new(1.0, 2.0, 3.0);
    /// let transformed_point = transform.transform_point(point);
    ///
    /// if let Some(inverse_transform) = transform.inverse() {
    ///     let original_point = inverse_transform.transform_point(transformed_point);
    ///     assert_eq!(original_point, point);
    /// }
    /// ```
    ///
    /// Note: The inverse transformation will only affect the X and Y coordinates of a `Vector3`. The Z coordinate remains unchanged.
    #[allow(dead_code)]
    pub fn inverse(&self) -> Option<RigidTransform2D<T>> {
        let denom = self.m00 * self.m11 - self.m01 * self.m10;
        if !denom.is_normal() {
            //println!("denom was 0.0 {}", denom);
            return self.inverse_determinant();
        }

        // Calculate the adjoint of A
        let inverse = RigidTransform2D {
            m00: self.m11 / denom,
            m01: -self.m01 / denom,
            m02: (self.m01 * self.m12 - self.m02 * self.m11) / denom,
            m10: -self.m10 / denom,
            m11: self.m00 / denom,
            m12: (-self.m00 * self.m12 + self.m02 * self.m10) / denom,
        };

        Some(inverse)
    }

    #[allow(dead_code)]
    pub fn inverse_determinant(&self) -> Option<RigidTransform2D<T>> {
        // Calculate the determinant of A
        let det = self.m00 * self.m11 - self.m01 * self.m10;

        if !det.abs().is_normal() {
            return None; // Singular matrix, cannot find its inverse
        }

        let inv_det = T::Scalar::ONE / det;
        // Calculate the adjoint of A
        // Multiply each element of the adjoint by 1/det

        let inverse = RigidTransform2D {
            m00: self.m11 * inv_det,
            m01: self.m01 * inv_det,
            m02: (self.m01 * self.m12 - self.m02 * self.m11) * inv_det,
            m10: -self.m10 * inv_det,
            m11: self.m00 * inv_det,
            m12: (self.m02 * self.m10 - self.m00 * self.m12) * inv_det,
        };

        Some(inverse)
    }
}

/*
#[derive(Clone, Copy, Debug)]
pub struct HashableVector2 {
    x: f32,
    y: f32,
}

impl HashableVector2 {
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}
*/
/// This is a simple convert trait intended to be used to convert within the same "family" of types.
/// E.g glam::Vec2 & glam::DVec2. It's not really intended to cover every possible combination of
/// conversions
/// It probably uses `as` to convert the numbers, so be warned.
pub trait ConvertTo<T> {
    fn to(self) -> T;
}

impl ConvertTo<DVec3> for DVec3 {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<Vec3> for Vec3 {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<Vec3A> for Vec3A {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<DVec3> for Vec3 {
    #[inline(always)]
    fn to(self) -> DVec3 {
        DVec3 {
            x: self.x as f64,
            y: self.y as f64,
            z: self.z as f64,
        }
    }
}

impl ConvertTo<DVec3> for Vec3A {
    #[inline(always)]
    fn to(self) -> DVec3 {
        DVec3 {
            x: self.x as f64,
            y: self.y as f64,
            z: self.z as f64,
        }
    }
}

impl ConvertTo<Vec3> for DVec3 {
    #[inline(always)]
    fn to(self) -> Vec3 {
        Vec3 {
            x: self.x as f32,
            y: self.y as f32,
            z: self.z as f32,
        }
    }
}

impl ConvertTo<Vec3A> for DVec3 {
    #[inline(always)]
    fn to(self) -> Vec3A {
        Vec3A {
            x: self.x as f32,
            y: self.y as f32,
            z: self.z as f32,
        }
    }
}

impl ConvertTo<Vec3A> for Vec3 {
    #[inline(always)]
    fn to(self) -> Vec3A {
        Vec3A {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

impl ConvertTo<Vec3> for Vec3A {
    #[inline(always)]
    fn to(self) -> Vec3 {
        Vec3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

impl ConvertTo<DVec2> for DVec2 {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<Vec2> for Vec2 {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<DVec2> for Vec2 {
    #[inline(always)]
    fn to(self) -> DVec2 {
        DVec2 {
            x: self.x as f64,
            y: self.y as f64,
        }
    }
}

impl ConvertTo<Vec2> for DVec2 {
    #[inline(always)]
    fn to(self) -> Vec2 {
        Vec2 {
            x: self.x as f32,
            y: self.y as f32,
        }
    }
}

/*
// some cgmath impls not needed right now
impl ConvertTo<Vector3<f64>> for Vector3<f64> {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<Vector3::<f32>> for Vector3<f32> {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<Vector3<f64>> for Vector3<f32> {
    #[inline(always)]
    fn to(self) -> Vector3<f64> {
        Vector3::<f64> {
            x: self.x as f64,
            y: self.y as f64,
            z: self.z as f64,
        }
    }
}

impl ConvertTo<Vector3<f32>> for Vector3<f64> {
    #[inline(always)]
    fn to(self) -> Vector3::<f32> {
        Vector3::<f32> {
            x: self.x as f32,
            y: self.y as f32,
            z: self.z as f32,
        }
    }
}

impl ConvertTo<Vector2<f64>> for Vector2<f64> {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<Vector2<f32>> for Vector2<f32> {
    #[inline(always)]
    fn to(self) -> Self {
        self
    }
}

impl ConvertTo<Vector2<f64>> for Vector2<f32> {
    #[inline(always)]
    fn to(self) -> Vector2<f64> {
        Vector2::<f64> {
            x: self.x as f64,
            y: self.y as f64,
        }
    }
}

impl ConvertTo<Vector2<f32>> for Vector2<f64> {
    #[inline(always)]
    fn to(self) -> Vector2<f32> {
        Vector2::<f32> {
            x: self.x as f32,
            y: self.y as f32,
        }
    }
}
*/
