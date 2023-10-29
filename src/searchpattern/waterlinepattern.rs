// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (c) 2023 lacklustr@protonmail.com https://github.com/eadf
// This file is part of the hronn crate.

// Concrete Strategy Waterline, Work in progress
pub struct WaterlinePattern {}

impl SearchPattern for WaterlinePattern {
    fn search(
        &self,
        _context: &MeshAnalyzer<'_>,
        _probe: &dyn Probe,
    ) -> Result<SearchResult, CollisionError> {
        // ... implementation for this pattern
        Err(CollisionError::InternalError("not implemented".to_string()))
    }
}
