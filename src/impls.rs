// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use super::meshanalyzer::SpatialTriangle;
use krakel::PointTrait;
use std::cmp::Ordering;
use std::hash::{Hash, Hasher};
use vector_traits::GenericVector2;

impl<T: GenericVector2> PointTrait for SpatialTriangle<T> {
    type PScalar = T::Scalar;
    #[inline(always)]
    fn x(&self) -> T::Scalar {
        self.center.x()
    }
    #[inline(always)]
    fn y(&self) -> T::Scalar {
        self.center.y()
    }
    #[inline(always)]
    fn set_x(&mut self, x: T::Scalar) {
        self.center.set_x(x)
    }
    #[inline(always)]
    fn set_y(&mut self, y: T::Scalar) {
        self.center.set_y(y)
    }
    #[inline(always)]
    fn at(&self, index: u8) -> T::Scalar {
        match index {
            0 => self.center.x(),
            1 => self.center.y(),
            _ => panic!("Invalid index for 2D point!"),
        }
    }
    #[inline(always)]
    fn at_mut(&mut self, index: u8) -> &mut T::Scalar {
        match index {
            0 => self.center.x_mut(),
            1 => self.center.y_mut(),
            _ => panic!("Invalid index for 2D point!"),
        }
    }

    const DIMENSION: u8 = 2;
}

impl<T: GenericVector2> PartialEq for SpatialTriangle<T> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}

impl<T> Eq for SpatialTriangle<T> where T: GenericVector2 {}

impl<T: GenericVector2> Hash for SpatialTriangle<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.index.hash(state);
    }
}

impl<T: GenericVector2> Ord for SpatialTriangle<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.index.cmp(&other.index)
    }
}

impl<T: GenericVector2> PartialOrd for SpatialTriangle<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.index.cmp(&other.index))
    }
}
