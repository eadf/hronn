// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use crate::HronnError;
use crate::HronnError::ParseFloatError;
use std::{
    fs::File,
    io::{BufRead, BufReader, Write},
    path,
};
use vector_traits::HasXYZ;

#[derive(Debug)]
pub struct Obj<MESH: HasXYZ> {
    pub name: String,
    pub vertices: Vec<MESH>,
    // the indices inside faces and line will be "real" indices, starting at zero.
    // 1 will be added to their value when the obj file is saved.
    // each triangle will be 3 indices
    pub indices: Vec<usize>,
    pub lines: Vec<Vec<usize>>,
}

#[allow(dead_code)]
impl<MESH: HasXYZ> Obj<MESH> {
    pub fn new(object_name: &str) -> Self {
        Self {
            name: object_name.to_string(),
            vertices: vec![],
            indices: vec![],
            lines: vec![],
        }
    }

    #[allow(dead_code)]
    pub fn new_from_triangles(object_name: &str, vertices: Vec<MESH>, indices: Vec<usize>) -> Self {
        Self {
            name: object_name.to_string(),
            vertices,
            indices,
            lines: vec![],
        }
    }

    pub fn add_vertex(&mut self, p0: MESH) -> usize {
        self.vertices.push(p0);
        self.vertices.len() - 1
    }

    /// inserts a new point in to self.lines. This point will be used to describe a line between
    /// this point and the previously inserted point in self.line
    pub fn continue_line(&mut self, point: MESH) {
        if self.lines.is_empty() {
            self.start_new_line(point);
        } else {
            let index = self.add_vertex(point);
            // we know last_mut() is Some
            self.lines.last_mut().unwrap().push(index);
        }
    }

    /// Works in conjunction with ´continue_line()´.
    /// Will take the first vertex and add to the end of the line
    /// Does nothing if no line is in progress
    pub fn close_line(&mut self) {
        if self.lines.is_empty() {
            if let Some(first_index) = self.lines.first().unwrap().first().cloned() {
                self.lines.last_mut().unwrap().push(first_index);
            }
        }
    }

    pub fn start_new_line(&mut self, point: MESH) {
        self.lines.push(Vec::new());
        let index = self.add_vertex(point);
        // we know last_mut() is Some
        self.lines.last_mut().unwrap().push(index);
    }

    #[allow(dead_code)]
    pub fn add_triangle(&mut self, p0: usize, p1: usize, p2: usize) {
        self.indices.push(p0);
        self.indices.push(p1);
        self.indices.push(p2);
    }

    #[allow(dead_code)]
    pub fn add_triangle_as_vertices(&mut self, p0: MESH, p1: MESH, p2: MESH) {
        let index = self.vertices.len();
        self.vertices.push(p0);
        self.vertices.push(p1);
        self.vertices.push(p2);
        self.indices.push(index);
        self.indices.push(index + 1);
        self.indices.push(index + 2);
    }

    pub fn write_obj(&self, filename: &std::path::Path) -> Result<(), HronnError> {
        let mut file = File::create(filename)?;
        if self.vertices.is_empty() {
            return Ok(());
        }

        let max_index = self.vertices.len() - 1;
        // Write object name
        writeln!(file, "o {}", self.name)?;

        // Write vertices
        for vertex in &self.vertices {
            //writeln!(file, "v {:e} {:e} {:e}", vertex.x, vertex.y, vertex.z)?;
            writeln!(file, "v {} {} {}", vertex.x(), vertex.y(), vertex.z())?;
        }

        // Write faces
        for face in self.indices.chunks(3) {
            write!(file, "f ")?;
            for element in face {
                if element > &max_index {
                    return Err(HronnError::InternalError(format!(
                        "the vertex index was too high {element} > {max_index}"
                    )));
                }
                // Remember, .obj uses 1-based indexing, so we add 1 to each index
                write!(file, "{} ", element + 1)?;
            }
            writeln!(file)?;
        }
        // Write lines
        for line in &self.lines {
            if !line.is_empty() {
                for window in line.windows(2) {
                    match window {
                        [a, b] => {
                            assert!(
                                a <= &max_index,
                                "the vertex index was too high {} > {}",
                                a,
                                max_index
                            );
                            assert!(
                                b <= &max_index,
                                "the vertex index was too high {} > {}",
                                b,
                                max_index
                            );
                            // Remember, .obj uses 1-based indexing, so we add 1 to each index
                            writeln!(file, "f {} {}", a + 1, b + 1)?;
                        }
                        _ => unreachable!(),
                    }
                }
            }
        }

        Ok(())
    }

    pub fn new_from_file(
        filename: impl AsRef<path::Path> + std::fmt::Debug,
    ) -> Result<Obj<MESH>, HronnError> {
        let file = File::open(filename.as_ref())?;
        let reader = BufReader::new(file);

        let mut name = String::new();
        let mut vertices: Vec<MESH> = Vec::new();
        let mut faces = Vec::new();

        for line in reader.lines() {
            let line = line?;
            let mut parts = line.split_whitespace();
            match parts.next() {
                Some("o") => {
                    name = parts.next().unwrap_or("").to_string();
                }
                Some("v") => {
                    let x: MESH::Scalar = parts
                        .next()
                        .ok_or_else(|| ParseFloatError)?
                        .parse()
                        .map_err(|_| ParseFloatError)?;
                    let y: MESH::Scalar = parts
                        .next()
                        .ok_or_else(|| ParseFloatError)?
                        .parse()
                        .map_err(|_| ParseFloatError)?;
                    let z: MESH::Scalar = parts
                        .next()
                        .ok_or_else(|| ParseFloatError)?
                        .parse()
                        .map_err(|_| ParseFloatError)?;
                    vertices.push(MESH::new_3d(x, y, z));
                }
                Some("f") => {
                    let face_indices: Vec<usize> = parts
                        .map(|part| part.split('/').next().unwrap_or("0"))
                        .filter_map(|s| s.parse().ok())
                        .collect();
                    if face_indices.len() == 3 {
                        faces.push(face_indices[0] - 1);
                        faces.push(face_indices[1] - 1);
                        faces.push(face_indices[2] - 1);
                    } else {
                        return Err(HronnError::NotTriangulated(format!(
                            "The file {:?} did not contain a triangulated mesh",
                            filename.as_ref()
                        )));
                    }
                    //faces.push(face_indices.iter().map(|v| v - 1).collect());
                }
                _ => {}
            }
        }

        Ok(Obj {
            name,
            vertices,
            indices: faces,
            lines: vec![],
        })
    }
}
