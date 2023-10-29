// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.
mod impls;

use crate::{HronnError, HronnError::InternalError};
use std::{fs::File, io::Write};
use vector_traits::HasXYZ;

#[derive(Clone, Debug)]
pub enum StrategyResult<MESH: HasXYZ> {
    MeshData(MeshData<MESH>),
    LineData(LineData<MESH>),
}

#[derive(Clone, Debug)]
pub struct MeshData<MESH: HasXYZ> {
    pub vertices: Vec<MESH>,
    pub indices: Vec<usize>,
}

#[derive(Clone, Debug)]
pub struct LineData<MESH: HasXYZ> {
    pub vertices: Vec<MESH>,
    pub lines: Vec<Vec<usize>>,
}

impl<MESH: HasXYZ> StrategyResult<MESH> {
    pub fn get_line_data(self) -> Result<LineData<MESH>, HronnError> {
        if let StrategyResult::LineData(linedata) = self {
            Ok(linedata)
        } else {
            Err(InternalError(
                "Wrong kind of data container found".to_string(),
            ))
        }
    }

    pub fn get_mesh_data(self) -> Result<MeshData<MESH>, HronnError> {
        if let StrategyResult::MeshData(meshdata) = self {
            Ok(meshdata)
        } else {
            Err(InternalError(
                "Wrong kind of data container found".to_string(),
            ))
        }
    }
}

impl<MESH: HasXYZ> MeshData<MESH> {
    pub fn new(vertices: Vec<MESH>, indices: Vec<usize>) -> Self {
        Self { vertices, indices }
    }

    pub fn add_vertex(&mut self, p0: MESH) -> usize {
        self.vertices.push(p0);
        self.vertices.len() - 1
    }

    pub fn add_triangle(&mut self, p0: usize, p1: usize, p2: usize) {
        self.indices.push(p0);
        self.indices.push(p1);
        self.indices.push(p2);
    }

    pub fn add_triangle_as_vertices(&mut self, p0: MESH, p1: MESH, p2: MESH) {
        let index = self.vertices.len();
        self.vertices.push(p0);
        self.vertices.push(p1);
        self.vertices.push(p2);
        self.indices.push(index);
        self.indices.push(index + 1);
        self.indices.push(index + 2);
    }

    /// saves the data as an rudimentary .obj file
    pub fn save_as_obj(
        &self,
        filename: &std::path::Path,
        obj_name: &str,
    ) -> Result<(), HronnError> {
        let mut file = File::create(filename)?;
        if self.vertices.is_empty() {
            return Ok(());
        }

        let max_index = self.vertices.len() - 1;
        // Write object name
        writeln!(file, "o {}", obj_name)?;

        // Write vertices
        for vertex in &self.vertices {
            writeln!(
                file,
                "v {:+e} {:+e} {:+e}",
                vertex.x(),
                vertex.y(),
                vertex.z()
            )?;
        }

        // Write faces
        for face in self.indices.chunks(3) {
            write!(file, "f ")?;
            for element in face {
                if element > &max_index {
                    return Err(InternalError(format!(
                        "the vertex index was too high {element} > {max_index}"
                    )));
                }
                // Remember, .obj uses 1-based indexing, so we add 1 to each index
                write!(file, "{} ", element + 1)?;
            }
            writeln!(file)?;
        }
        Ok(())
    }
}

impl<MESH: HasXYZ> LineData<MESH> {
    pub fn add_vertex(&mut self, p0: MESH) -> usize {
        self.vertices.push(p0);
        self.vertices.len() - 1
    }

    /// inserts a new point into self.lines. This point will be used to describe a line between
    /// this point and the previously inserted point
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

    /// saves the data as an rudimentary .obj file
    pub fn save_as_obj(
        &self,
        filename: &std::path::Path,
        obj_name: &str,
    ) -> Result<(), HronnError> {
        let mut file = File::create(filename)?;
        if self.vertices.is_empty() {
            return Ok(());
        }

        let max_index = self.vertices.len() - 1;
        // Write object name
        writeln!(file, "o {}", obj_name)?;

        // Write vertices
        for vertex in &self.vertices {
            writeln!(file, "v {} {} {}", vertex.x(), vertex.y(), vertex.z())?;
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
}
