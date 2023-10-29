// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use crate::triangulation::DelaunayPos;
use std::{
    fmt,
    fmt::{Debug, Display},
};
use vector_traits::{num_traits::real::Real, GenericVector3};
/*
impl<T: GenericVector3> From<MyVec2_5<T>> for T {
    fn from(vec: MyVec2_5<T>) -> Self {
        vec.0
    }
}*/

impl<T: GenericVector3> Debug for DelaunayPos<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fn format_float<F: Real + Display>(value: F) -> String {
            if value.fract().is_zero() {
                format!("{:.1}", value)
            } else {
                format!("{}", value)
            }
        }

        write!(
            f,
            "({},{},{})",
            format_float(self.0.x()),
            format_float(self.0.y()),
            format_float(self.0.z()),
        )
    }
}
