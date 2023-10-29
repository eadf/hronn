// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

use vector_traits::num_traits;

pub struct MaximumTracker<T> {
    current_max: Option<T>,
}

pub trait MaximumTrackerInsertable<T: PartialOrd + Copy> {
    fn insert_into(self, current_max: &mut Option<T>);
}

impl<T: PartialOrd + Copy> MaximumTrackerInsertable<T> for T {
    #[inline(always)]
    fn insert_into(self, current_max: &mut Option<T>) {
        match *current_max {
            Some(ref mut max_value) if self > *max_value => *max_value = self,
            None => *current_max = Some(self),
            _ => {}
        }
    }
}

// Implement the trait for Option<T>
impl<T: PartialOrd + Copy> MaximumTrackerInsertable<T> for Option<T> {
    #[inline(always)]
    fn insert_into(self, current_max: &mut Option<T>) {
        if let Some(new_item) = self {
            new_item.insert_into(current_max);
        }
    }
}

impl<T: PartialOrd + Copy> Default for MaximumTracker<T> {
    fn default() -> Self {
        Self { current_max: None }
    }
}

impl<T: PartialOrd + Copy> MaximumTracker<T> {
    pub fn new(value: T) -> Self {
        MaximumTracker {
            current_max: Some(value),
        }
    }

    #[inline(always)]
    pub fn insert<U: MaximumTrackerInsertable<T>>(&mut self, item: U) {
        item.insert_into(&mut self.current_max);
    }

    #[inline(always)]
    pub fn get_max(&self) -> Option<T> {
        self.current_max
    }
}

pub(crate) type VobU32 = vob::Vob<u32>;

pub(crate) trait GrowingVob {
    /// Will create a new Vob and fill it with `false`
    fn fill_with_false(initial_size: usize) -> Self;
    /// Conditionally grow to fit required size, set ´bit´ to ´state´ value
    fn grow_and_set(&mut self, bit: usize, state: bool);
    /// get() with default value `false`
    fn get_or_false(&self, bit: usize) -> bool;
    fn set_false(&mut self, bit: usize);
    fn set_true(&mut self, bit: usize);
}

impl<T: num_traits::PrimInt + std::fmt::Debug> GrowingVob for vob::Vob<T> {
    #[inline]
    fn fill_with_false(initial_size: usize) -> Self {
        let mut v = Self::new_with_storage_type(0);
        v.resize(initial_size, false);
        v
    }

    #[inline]
    fn grow_and_set(&mut self, bit: usize, state: bool) {
        if bit >= self.len() {
            self.resize(bit + std::mem::size_of::<T>(), false);
        }
        let _ = self.set(bit, state);
    }

    #[inline(always)]
    fn get_or_false(&self, bit: usize) -> bool {
        self.get(bit).unwrap_or(false)
    }

    #[inline(always)]
    fn set_false(&mut self, bit: usize) {
        let _ = self.set(bit, false);
    }

    #[inline(always)]
    fn set_true(&mut self, bit: usize) {
        let _ = self.set(bit, true);
    }
}
