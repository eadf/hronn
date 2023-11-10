// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

//! A module containing boiler-plate implementations of standard traits such as Default, Debug etc etc

use super::*;
use vector_traits::{GenericVector2, GenericVector3};
impl<T: GenericVector2> Debug for Circle<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Sphere(c:{:?},r:{})", self.center, self.radius)
    }
}

impl<T> Default for RigidTransform2D<T>
where
    T: GenericVector3,
{
    fn default() -> Self {
        Self::identity()
    }
}

impl<T> Debug for RigidTransform2D<T>
where
    T: GenericVector3,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fn format_float<F>(value: F) -> String
        where
            F: Float + Display,
        {
            if value.fract().is_zero() {
                format!("{:.1}", value)
            } else {
                format!("{}", value)
            }
        }

        write!(
            f,
            "(({},{},{}))\n({},{},{}))\n({},{},{})))",
            format_float(self.m00),
            format_float(self.m01),
            format_float(self.m02),
            format_float(self.m10),
            format_float(self.m11),
            format_float(self.m12),
            0.0,
            0.0,
            1.1
        )
    }
}
