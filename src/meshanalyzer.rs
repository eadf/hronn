// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.
mod impls;

use super::ProbeMode;
use crate::{
    //ffi::FFIVector3,
    geo,
    geo::ConvertTo,
    obj,
    util::{GrowingVob, MaximumTracker, VobU32},
    HronnError,
};
use krakel::KDTree;
use std::{fmt::Debug, path::Path};
use vector_traits::{
    approx, num_traits::real::Real, GenericScalar, GenericVector2, GenericVector3, HasXYZ,
};

#[derive(Clone, Debug)]
pub(crate) struct SpatialTriangle<T: GenericVector2> {
    pub(crate) center: T,
    // a reference back into the MeshAnalyzer::vertices
    pub(crate) index: usize,
}
impl<T: GenericVector2> SpatialTriangle<T> {
    pub(crate) fn new(index: usize, center: T) -> Self {
        Self { center, index }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SearchResult<T: GenericVector3> {
    pub(crate) z_value: T::Scalar,
    // a reference back into the MeshAnalyzer::vertices
    pub(crate) index: usize,
}

impl<T: GenericVector3> SearchResult<T> {
    #[allow(dead_code)]
    pub(crate) fn new(index: usize, z_value: T::Scalar) -> Self {
        Self { z_value, index }
    }
}

pub struct MeshAnalyzerBuilder<'a, T: GenericVector3, MESH: HasXYZ>
where
    T::Scalar: approx::UlpsEq,
    MESH: ConvertTo<T>,
    T: ConvertTo<MESH>,
{
    data_vertices: Option<Data<'a, MESH>>,
    data_indices: Option<Data<'a, usize>>,
    split_distance: Option<T::Scalar>,
}

impl<'a, T: GenericVector3, MESH: HasXYZ> MeshAnalyzerBuilder<'a, T, MESH>
where
    MESH: ConvertTo<T> + 'a,
    T: ConvertTo<MESH>,
{
    /// Insert the mesh as owned data from an .obj file.
    pub fn load_from_obj(mut self, filename: &Path) -> Result<Self, HronnError> {
        let obj = obj::Obj::new_from_file(filename)?;

        self.data_vertices = Some(Data::Owned(obj.vertices));
        self.data_indices = Some(Data::Owned(obj.indices));
        Ok(self)
    }

    /// Insert the mesh as references.
    pub fn load_from_ref(
        mut self,
        vertices: &'a [MESH],
        indices: &'a [usize],
    ) -> Result<Self, HronnError> {
        self.data_vertices = Some(Data::Ref(vertices));
        self.data_indices = Some(Data::Ref(indices));
        Ok(self)
    }

    /// Add the command to split the triangles in the mesh if they have sides shorter
    /// than split_distance
    pub fn with_split(mut self, split_distance: T::Scalar) -> Result<Self, HronnError> {
        self.split_distance = Some(split_distance);
        Ok(self)
    }

    /// Finalizes the build of the MeshAnalyzer
    pub fn build(self) -> Result<MeshAnalyzer<'a, T, MESH>, HronnError> {
        if self.data_vertices.is_none() {
            return Err(HronnError::InternalError("missing vertices".to_string()));
        }
        if self.data_indices.is_none() {
            return Err(HronnError::InternalError("missing indices".to_string()));
        }

        let ma =
            MeshAnalyzer::<T, MESH>::new(self.data_vertices.unwrap(), self.data_indices.unwrap())?;
        let ma = if let Some(split_distance) = self.split_distance {
            ma.split_and_update_kd_tree(split_distance)?
        } else {
            ma.update_kd_tree()?
        };
        Ok(ma)
    }
}

/// An enum representing ownership of data.
///
/// `Data` can either own its contained data (`Owned`) or borrow it (`Ref`).
/// This is useful in scenarios where a structure might sometimes take ownership
/// of its data and at other times borrow it, depending on the context.
///
/// # Variants
///
/// - `Owned`: The data is owned by this enum variant.
/// - `Ref`: The data is borrowed, with a given lifetime `'a`.
///
/// # Examples
///
/// ```rust,ignore
/// let owned_data: Data<i32> = Data::Owned(vec![1, 2, 3]);
/// let borrowed_data: Data<i32> = Data::Ref(&[4, 5, 6]);
/// ```
pub(crate) enum Data<'a, T> {
    Owned(Vec<T>),
    Ref(&'a [T]),
}

pub struct MeshAnalyzer<'a, T: GenericVector3, MESH: HasXYZ> {
    pub(crate) triangle_side_length: T::Scalar,
    pub(crate) vertices: Data<'a, MESH>,
    pub(crate) indices: Data<'a, usize>,
    pub(crate) kd_tree: KDTree<SpatialTriangle<T::Vector2>>,
    pub(crate) mode: ProbeMode,
}

struct SplitContainer<MESH: HasXYZ> {
    new_vertices: Vec<MESH>,
    split_triangles: Vec<([usize; 3], usize)>,
    split_indicator: VobU32,
}

impl<'a, T: GenericVector3, MESH: HasXYZ> MeshAnalyzer<'a, T, MESH>
where
    MESH: ConvertTo<T> + 'a,
    T: ConvertTo<MESH>,
{
    /// creates a new MeshAnalyzer with data that can be either owned or just referenced.
    fn new(vertices: Data<'a, MESH>, indices: Data<'a, usize>) -> Result<Self, HronnError> {
        let kd_tree = KDTree::<SpatialTriangle<T::Vector2>>::default();
        let vertices_ref = vertices.as_ref();
        let indices_ref = indices.as_ref();

        if vertices_ref.is_empty() {
            return Err(HronnError::NoData("No triangles found".to_string()));
        }
        let mut max_l_sq = MaximumTracker::<T::Scalar>::default();
        indices_ref.chunks(3).for_each(|triangle| {
            Self::max_side_sq(vertices_ref, triangle, &mut max_l_sq);
        });
        let triangle_side_length = max_l_sq.get_max().unwrap_or(T::Scalar::ZERO).sqrt();
        println!(
            "Created a non-split MeshAnalyzer object with and triangle side length={triangle_side_length}."
        );
        Ok(Self {
            vertices,
            indices,
            triangle_side_length,
            kd_tree,
            mode: ProbeMode::OriginalTriangles,
        })
    }

    /// simply update the kd-tree with the triangles we know about, no alterations
    fn update_kd_tree(mut self) -> Result<Self, HronnError> {
        let vertices_ref = self.vertices.as_ref();

        for (index, triangle) in self.indices.as_ref().chunks(3).enumerate() {
            let p0 = vertices_ref[triangle[0]];
            let p1 = vertices_ref[triangle[1]];
            let p2 = vertices_ref[triangle[2]];

            let center: T::Vector2 = geo::centroid_2d(
                <MESH as ConvertTo<T>>::to(p0).to_2d(),
                <MESH as ConvertTo<T>>::to(p1).to_2d(),
                <MESH as ConvertTo<T>>::to(p2).to_2d(),
            );

            self.kd_tree.insert(SpatialTriangle::new(index, center))?;
        }
        Ok(self)
    }

    /// Split the known triangles if any of their sides are too long into "virtual" triangles that
    /// we then insert into the kd-tree (together with the triangles that were short enough).
    /// These split triangles will point back to the original triangle, so the collision math is
    /// done against those.
    fn split_and_update_kd_tree(mut self, split_length: T::Scalar) -> Result<Self, HronnError> {
        let original_indices = self.indices.as_ref();
        let _original_indices_len = original_indices.len();
        let original_vertices = self.vertices.as_ref();
        let original_vertices_len = original_vertices.len();

        //let number_of_original_vertices = self.vertices.len();
        let mut _triangle_side_length: T::Scalar;

        // We are splitting the triangles in the mesh so that they are all of max length<=radius
        // Thus we can search by a distance of r+l/2 = 1.5*r
        let SplitContainer {
            new_vertices: added_vertices,
            split_triangles: added_triangles,
            split_indicator: was_split,
        } = Self::split_triangulated_faces(split_length, original_vertices, self.indices.as_ref());
        self.mode = if !added_triangles.is_empty() {
            println!(
                "Created a new MeshAnalyzer from {} vertices, split into {} additional vertices",
                original_vertices_len,
                added_vertices.len()
            );
            _triangle_side_length = split_length;
            ProbeMode::SplitTriangles
        } else {
            println!(
                "Created a new MeshAnalyzer from {} vertices",
                original_vertices_len
            );
            ProbeMode::OriginalTriangles
        };

        let get_vertex = |id: usize| {
            if id >= original_vertices_len {
                added_vertices[id - original_vertices_len]
            } else {
                original_vertices[id]
            }
        };

        original_indices
            .chunks(3)
            .enumerate()
            .for_each(|(id, triangle)| {
                let p0 = get_vertex(triangle[0]);
                let p1 = get_vertex(triangle[1]);
                let p2 = get_vertex(triangle[2]);

                if !was_split[id] {
                    let center: T::Vector2 = geo::centroid_2d(
                        <MESH as ConvertTo<T>>::to(p0).to_2d(),
                        <MESH as ConvertTo<T>>::to(p1).to_2d(),
                        <MESH as ConvertTo<T>>::to(p2).to_2d(),
                    );
                    // only insert split triangles, or triangles already small enough
                    if false {
                        println!(
                            "inserted original center:{:?} id:{} [{},{},{}]",
                            center, id, triangle[0], triangle[1], triangle[2]
                        );
                    }
                    self.kd_tree
                        .insert(SpatialTriangle::new(id, center))
                        .unwrap();
                }
            });

        added_triangles.iter().for_each(|(triangle, id)| {
            let p0 = get_vertex(triangle[0]);
            let p1 = get_vertex(triangle[1]);
            let p2 = get_vertex(triangle[2]);

            let center: T::Vector2 = geo::centroid_2d(
                <MESH as ConvertTo<T>>::to(p0).to_2d(),
                <MESH as ConvertTo<T>>::to(p1).to_2d(),
                <MESH as ConvertTo<T>>::to(p2).to_2d(),
            );
            if false {
                println!(
                    "inserted added center:{:?} id:{} [{},{},{}]",
                    center, id, triangle[0], triangle[1], triangle[2]
                );
            }
            self.kd_tree
                .insert(SpatialTriangle::new(*id, center))
                .unwrap();
        });

        println!("Created a split MeshAnalyzer object with max triangle length={split_length}");
        Ok(self)
    }

    /// split existing triangles if their length is too long.
    /// Warning: this assumes the Obj data is triangulated. It does nothing for self::line
    fn split_triangulated_faces(
        max_length: T::Scalar,
        original_vertices: &[MESH],
        original_triangles: &[usize],
    ) -> SplitContainer<MESH> {
        if false {
            println!("original_triangles:{:?}", original_triangles);
        }
        let max_length_sq = max_length * max_length;
        let mut split_indicator = VobU32::fill_with_false(original_triangles.len());

        let candidate_triangles: Vec<_> = original_triangles
            .chunks(3)
            .enumerate()
            .filter_map(|(i, v)| {
                let mut max_l_sq = MaximumTracker::<T::Scalar>::default();
                Self::max_side_sq(original_vertices, v, &mut max_l_sq);
                if max_l_sq.get_max().unwrap() > max_length_sq {
                    let _ = split_indicator.set(i, true);
                    Some(([v[0], v[1], v[2]], i))
                } else {
                    None
                }
            })
            .collect();
        if false {
            println!("candidate_triangles.len()={}", candidate_triangles.len());
            println!("candidate_triangles:{:?}", candidate_triangles);
        }

        let (new_vertices, split_triangles) = Self::split_triangles(
            original_vertices,
            candidate_triangles,
            max_length * max_length,
        );
        if false {
            println!(
                "split_triangles:{:?} {}",
                split_triangles,
                split_triangles.len()
            );
            let mut vertices = original_vertices.to_vec();
            vertices.extend(&new_vertices);

            let split_triangles: Vec<_> = split_triangles
                .iter()
                .flat_map(|v| [v.0[0], v.0[1], v.0[2]])
                .collect();
            println!(
                "split_triangles:{:?} {}",
                split_triangles,
                split_triangles.len() / 3
            );
            let split_obj = obj::Obj::new_from_triangles("split", vertices, split_triangles);
            let _ = split_obj.write_obj(&Into::<std::path::PathBuf>::into("split.obj"));
            println!("wrote split.obj");
        }
        SplitContainer {
            new_vertices,
            split_triangles,
            split_indicator,
        }
    }

    fn split_triangles(
        original_vertices: &[MESH],
        mut triangles_to_split: Vec<([usize; 3], usize)>,
        max_length_sq: T::Scalar,
    ) -> (Vec<MESH>, Vec<([usize; 3], usize)>) {
        fn split_edge<T, MESH>(v1: &T, v2: &T, vertices: &mut Vec<MESH>) -> usize
        where
            T: GenericVector3 + ConvertTo<MESH>,
            MESH: HasXYZ + ConvertTo<T>,
        {
            let midpoint = (*v1 + *v2) / T::Scalar::TWO;
            //println!("created a new vertex: {:?}", midpoint);
            vertices.push(midpoint.to());
            vertices.len() - 1 // Return the index of the new vertex.
        }

        let original_vertices_len = original_vertices.len();

        if original_vertices.is_empty() {
            return (Vec::new(), triangles_to_split);
        }

        if false {
            println!(
                "Splitting triangles with sides longer than {}",
                max_length_sq.sqrt()
            );
        }
        let get_vertex = |id: usize, ov: &[MESH], av: &[MESH]| {
            if id >= original_vertices_len {
                av[id - original_vertices_len].to()
            } else {
                ov[id].to()
            }
        };
        let mut added_vertices: Vec<MESH> = Vec::new();
        let mut i = 0;
        while i < triangles_to_split.len() {
            let (triangle, origin) = triangles_to_split[i];
            //println!("checking face: {:?}", face);

            let v0 = get_vertex(triangle[0], original_vertices, &added_vertices);
            let v1 = get_vertex(triangle[1], original_vertices, &added_vertices);
            let v2 = get_vertex(triangle[2], original_vertices, &added_vertices);

            let d01 = v0.distance_sq(v1);
            let d12 = v1.distance_sq(v2);
            let d20 = v2.distance_sq(v0);

            let longest_edge = if d01 >= d12 && d01 >= d20 {
                (v0, v1, triangle[0], triangle[1], triangle[2])
            } else if d12 >= d01 && d12 >= d20 {
                (v1, v2, triangle[1], triangle[2], triangle[0])
            } else {
                (v2, v0, triangle[2], triangle[0], triangle[1])
            };
            //let max_length_sq = T::Scalar::to_f32(max_length_sq);
            if d01 > max_length_sq || d12 > max_length_sq || d20 > max_length_sq {
                let new_vertex_id =
                    split_edge::<T, MESH>(&longest_edge.0, &longest_edge.1, &mut added_vertices)
                        + original_vertices_len;
                let new_face0 = ([longest_edge.2, new_vertex_id, longest_edge.4], origin);
                let new_face1 = ([new_vertex_id, longest_edge.3, longest_edge.4], origin);
                if false {
                    println!(
                        "i:{} o:{} splitting face:{:?} longest edge:{:?} new_vertex_id:{}",
                        i, origin, triangle, longest_edge, new_vertex_id
                    );
                }

                // Replace the current triangle with the two new triangles
                triangles_to_split[i] = new_face0;
                triangles_to_split.push(new_face1);
            } else {
                i += 1;
            }
        }
        (added_vertices, triangles_to_split)
    }

    /// calculates the maximum squared length of the triangle sides
    fn max_side_sq(
        vertices: &[MESH],
        indices: &[usize],
        max_lenght: &mut MaximumTracker<T::Scalar>,
    ) {
        for i in 0..3 {
            let i1 = i;
            let i2 = (i + 1) % 3;
            max_lenght.insert(
                vertices[indices[i1]]
                    .to()
                    .distance_sq(vertices[indices[i2]].to()),
            );
        }
    }
}
