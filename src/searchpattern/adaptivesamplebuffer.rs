// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use crate::prelude::ConvertTo;
use std::collections::VecDeque;
use vector_traits::{
    approx::{AbsDiffEq, UlpsEq},
    num_traits::real::Real,
    Approx, GenericScalar, GenericVector2, GenericVector3, HasXYZ,
};

pub(super) struct AdaptiveSampleBuffer<T: GenericVector3, MESH: HasXYZ, F> {
    buffer: VecDeque<MESH>,
    // max size of the buffer
    capacity: usize,
    // the min distance in xy we allow for adaptive re-sampling, we stop sampling after this distance
    xy_dist_criteria_sq: T::Scalar,
    // How steep inclination in Z we allow between two samples
    z_dist_criteria: T::Scalar,
    samples: Vec<MESH>,
    sample_fn: F,
}

impl<T: GenericVector3, MESH: HasXYZ, F> AdaptiveSampleBuffer<T, MESH, F>
where
    T: ConvertTo<MESH>,
    MESH: ConvertTo<T> + Approx,
    F: Fn(T::Vector2) -> MESH,
{
    pub(super) fn new(
        capacity: usize,
        xy_dist_criteria_sq: T::Scalar,
        z_dist_criteria: T::Scalar,
        sample_fn: F,
    ) -> Self {
        Self {
            buffer: VecDeque::with_capacity(capacity),
            capacity,
            xy_dist_criteria_sq,
            z_dist_criteria,
            samples: Vec::new(),
            sample_fn,
        }
    }

    pub(super) fn push_sample(&mut self, sample: MESH) {
        if self.buffer.len() == self.capacity {
            let old_sample = self.buffer.pop_front().unwrap();
            self.samples.push(old_sample);
            self.buffer.push_back(sample);
        } else {
            self.buffer.push_back(sample);
        }
    }

    pub(super) fn sample_and_refine_reduce(&mut self, pos: T::Vector2) {
        self.push_sample((self.sample_fn)(pos));
        let mut i = 0;
        let mut j = 0;
        while i < self.buffer.len() - 1 {
            j += 1;
            if j > 100 {
                println!("+++++++++++++ sample_and_refine 0 is out of control");
                return;
            }
            let sample1 = self.buffer[i].to();
            let sample2 = self.buffer[i + 1].to();

            if (sample1.z() - sample2.z()).abs() > self.z_dist_criteria {
                // Check if we can refine further between these two samples
                let sample1 = sample1.to_2d();
                let sample2 = sample2.to_2d();
                if sample1.distance_sq(sample2) > self.xy_dist_criteria_sq {
                    // Place a new sample between them
                    let new_sample_pos = (sample1 + sample2) / T::Scalar::TWO;
                    let new_sample = (self.sample_fn)(new_sample_pos);

                    // Insert new sample into the buffer
                    self.buffer.insert(i + 1, new_sample);
                } else {
                    // If we've refined too much, just move to the next sample
                    i += 1;
                }
            } else {
                // If the condition isn't met, just move to the next sample
                i += 1;
            }
        }
        // Cleanup loop for removing middle sample of three identical Z values
        i = 0;
        j = 0;

        while self.buffer.len() >= 2 && i < self.buffer.len() - 2 {
            j += 1;
            if j > 100 {
                println!("+++++++++++++ sample_and_refine 2 is out of control");
                return;
            }
            if self.buffer[i].z() == self.buffer[i + 1].z()
                && self.buffer[i + 1].z() == self.buffer[i + 2].z()
            {
                let _ = self.buffer.remove(i + 1); // Remove the middle one
            } else {
                i += 1;
            }
        }
        j = 0;
        // Check and drain the buffer if it exceeds the capacity
        if self.buffer.len() > self.capacity {
            let overflow = self.buffer.len() - self.capacity;
            let epsilon = MESH::Scalar::default_epsilon();
            let ulp = MESH::Scalar::default_max_ulps();
            for _ in 0..overflow {
                j += 1;
                if j > 100 {
                    println!("+++++++++++++ sample_and_refine 3 is out of control");
                    return;
                }
                let sample = self.buffer.pop_front().unwrap();
                if let Some(last) = self.samples.last() {
                    if last.is_ulps_eq(sample, epsilon, ulp) {
                        println!("++ identical sample found: {:?}", sample);
                        continue;
                    }
                }
                self.samples.push(sample);
            }
        }
    }
    pub(super) fn sample_and_refine(&mut self, pos: T::Vector2) {
        self.push_sample((self.sample_fn)(pos));
        let mut i = 0;
        let mut j = 0;
        let epsilon = MESH::Scalar::default_epsilon();
        let ulp = MESH::Scalar::default_max_ulps();
        while i < self.buffer.len() - 1 {
            j += 1;
            if j > 100 {
                println!("+++++++++++++ sample_and_refine 0 is out of control");
                return;
            }
            let sample1 = self.buffer[i].to();
            let sample2 = self.buffer[i + 1].to();

            if (sample1.z() - sample2.z()).abs() > self.z_dist_criteria {
                // Check if we can refine further between these two samples
                let sample1 = sample1.to_2d();
                let sample2 = sample2.to_2d();
                if sample1.distance_sq(sample2) > self.xy_dist_criteria_sq {
                    // Place a new sample between them
                    let new_sample_pos = (sample1 + sample2) / T::Scalar::TWO;
                    let new_sample = (self.sample_fn)(new_sample_pos);

                    // Insert new sample into the buffer
                    self.buffer.insert(i + 1, new_sample);
                } else {
                    // If we've refined too much, just move to the next sample
                    i += 1;
                }
            } else {
                // If the condition isn't met, just move to the next sample
                i += 1;
            }
        }
        j = 0;
        // Check and drain the buffer if it exceeds the capacity
        if self.buffer.len() > self.capacity {
            let overflow = self.buffer.len() - self.capacity;
            for _ in 0..overflow {
                j += 1;
                if j > 100 {
                    println!("+++++++++++++ sample_and_refine 3 is out of control");
                    return;
                }
                let sample = self.buffer.pop_front().unwrap();
                if let Some(last) = self.samples.last() {
                    if last.is_ulps_eq(sample, epsilon, ulp) {
                        println!("++ identical sample found: {:?}", sample);
                        continue;
                    }
                }
                self.samples.push(sample);
            }
        }
    }

    pub(super) fn drain(&mut self) {
        self.samples.extend(self.buffer.drain(..));
    }

    pub(super) fn finalize(mut self) -> Vec<MESH> {
        self.samples.extend(self.buffer.drain(..));
        self.samples
    }
}
