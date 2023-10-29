// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.
mod impls;
#[cfg(test)]
mod tests;
use crate::{prelude::ConvertTo, HronnError};
use linestring::linestring_2d::Aabb2;
use spade::{
    handles::FixedVertexHandle, DelaunayTriangulation, HasPosition, Point2, Triangulation,
};
use vector_traits::{
    num_traits::AsPrimitive, GenericScalar, GenericVector2, GenericVector3, HasXYZ,
};

/// This represents a 2.5d point, used with the space crate.
pub struct DelaunayPos<T: HasXYZ>(T);

impl<T: HasXYZ> HasPosition for DelaunayPos<T> {
    type Scalar = T::Scalar;

    fn position(&self) -> Point2<Self::Scalar> {
        Point2::new(self.0.x(), self.0.y())
    }
}

pub(super) struct DelaunayContainer<T: HasXYZ> {
    pub(super) delaunay: DelaunayTriangulation<DelaunayPos<T>>,
    pub(super) vertices: Vec<T>,
    pub(super) indices: Vec<usize>,
}

impl<T: HasXYZ> DelaunayContainer<T> {
    pub(super) fn insert_vertex(&mut self, vertex: T) -> Result<FixedVertexHandle, HronnError> {
        let inserted_vertex = self.delaunay.insert(DelaunayPos(vertex))?;
        /*println!(
            "inserted {:?} as real:{}, offset:{}",
            vertex,
            inserted_vertex.index(),
            self.vertices.len()
        );*/
        if inserted_vertex.index() > 2 && inserted_vertex.index() - 3 == self.vertices.len() {
            self.vertices.push(vertex);
        }
        Ok(inserted_vertex)
    }

    pub(super) fn insert_vertex_with_hint(
        &mut self,
        vertex: T,
        hint: FixedVertexHandle,
    ) -> Result<FixedVertexHandle, HronnError> {
        let inserted_vertex = self.delaunay.insert_with_hint(DelaunayPos(vertex), hint)?;
        /*println!(
            "inserted {:?} as real:{}, offset:{}",
            vertex,
            inserted_vertex.index(),
            self.vertices.len()
        );*/
        if inserted_vertex.index() > 2 && inserted_vertex.index() - 3 == self.vertices.len() {
            self.vertices.push(vertex);
        }
        Ok(inserted_vertex)
    }
}

/// Does sampling across a mesh then does triangulation on the result
pub fn triangulate_vertices<T: GenericVector3, MESH: HasXYZ>(
    aabb2: Aabb2<T::Vector2>,
    convex_hull: &[T::Vector2],
    vertices: &[MESH],
) -> Result<(Vec<MESH>, Vec<usize>), HronnError>
where
    T: ConvertTo<MESH>,
    T::Scalar: AsPrimitive<MESH::Scalar>,
{
    if let Some((min, max, width, height)) = aabb2.extents() {
        let min = min.to_3d(T::Scalar::ZERO).to();
        let max = max.to_3d(T::Scalar::ZERO).to();
        let width: MESH::Scalar = width.as_();
        let width = width * 10.0.into();
        let height: MESH::Scalar = height.as_();
        let height = height * 10.0.into();

        let big_triangle_a = MESH::new_3d(min.x() - width, min.y() - height, MESH::Scalar::ZERO);
        let big_triangle_b = MESH::new_3d(max.x() + width, min.y() - height, MESH::Scalar::ZERO);
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
            vertices: Vec::<MESH>::with_capacity(4 + convex_hull.len() + vertices.len()),
            indices: Vec::<usize>::with_capacity((4 + convex_hull.len() + vertices.len()) * 3),
        };

        let mut last_vertex = tris.insert_vertex(big_triangle_a)?;
        last_vertex = tris.insert_vertex_with_hint(big_triangle_b, last_vertex)?;
        let _ = tris.insert_vertex_with_hint(big_triangle_c, last_vertex)?;

        last_vertex =
            tris.insert_vertex(convex_hull.first().unwrap().to_3d(T::Scalar::ZERO).to())?;
        for v in convex_hull.iter().skip(1) {
            last_vertex =
                tris.insert_vertex_with_hint(v.to_3d(T::Scalar::ZERO).to(), last_vertex)?;
        }
        // assume that each vertex is close to the last
        let mut last_vertex = tris.insert_vertex(*vertices.first().unwrap())?;
        for v in vertices.iter().skip(1) {
            last_vertex = tris.insert_vertex_with_hint(*v, last_vertex)?;
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
