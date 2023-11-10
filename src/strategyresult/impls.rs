// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.
//! A module containing boiler-plate implementations of standard traits such as Default, From etc etc

use crate::prelude::{LineData, MeshData};
use vector_traits::HasXYZ;

impl<MESH: HasXYZ> Default for LineData<MESH> {
    fn default() -> Self {
        Self {
            vertices: Vec::<MESH>::default(),
            lines: Vec::<Vec<usize>>::default(),
        }
    }
}

impl<MESH: HasXYZ> Default for MeshData<MESH> {
    fn default() -> Self {
        Self {
            vertices: Vec::<MESH>::default(),
            indices: Vec::<usize>::default(),
        }
    }
}
