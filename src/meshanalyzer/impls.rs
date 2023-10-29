// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use super::{Data, MeshAnalyzer};
use crate::{geo::ConvertTo, meshanalyzer::SearchResult, prelude::MeshAnalyzerBuilder};
use vector_traits::{approx, GenericVector3, HasXYZ};

impl<T: GenericVector3, MESH: HasXYZ> PartialEq for &MeshAnalyzer<'_, T, MESH>
where
    T::Scalar: approx::UlpsEq,
    MESH: ConvertTo<T>,
{
    fn eq(&self, other: &Self) -> bool {
        // Compare the references by their raw pointer values
        std::ptr::eq(*self, *other)
    }
}

impl<T: GenericVector3, MESH: HasXYZ> Eq for &MeshAnalyzer<'_, T, MESH> where MESH: ConvertTo<T> {}

impl<'a, T> AsRef<[T]> for Data<'a, T> {
    fn as_ref(&self) -> &[T] {
        match self {
            Data::Owned(v) => v,
            Data::Ref(slice) => slice,
        }
    }
}

impl<T: GenericVector3> PartialEq for SearchResult<T> {
    fn eq(&self, other: &Self) -> bool {
        self.z_value.eq(&other.z_value)
    }
}

impl<T: GenericVector3> PartialOrd for SearchResult<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        match self.z_value.partial_cmp(&other.z_value) {
            Some(std::cmp::Ordering::Equal) => self.index.partial_cmp(&other.index),
            other => other,
        }
    }
}

impl<T: GenericVector3, MESH: HasXYZ> Default for MeshAnalyzerBuilder<'_, T, MESH>
where
    T::Scalar: approx::UlpsEq,
    MESH: ConvertTo<T>,
    T: ConvertTo<MESH>,
{
    fn default() -> Self {
        Self {
            data_vertices: None,
            data_indices: None,
            split_distance: None,
        }
    }
}
