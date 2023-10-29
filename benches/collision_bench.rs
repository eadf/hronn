use hronn::prelude::*;

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use linestring::linestring_2d::{Aabb2, LineString2};
use rand::{rngs::StdRng, Rng, SeedableRng};
use std::{env, path::Path};
use vector_traits::glam::{DVec2, DVec3, Vec3};

fn original_triangles_large(c: &mut Criterion) {
    let probe_radius = 1.0;
    let minimum_z = 0.0;
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let filename = Path::new(&manifest_dir).join("benches/sample.obj");
    let mesh_analyzer = MeshAnalyzerBuilder::<DVec3, Vec3>::default()
        .load_from_obj(&filename)
        .unwrap()
        .build()
        .unwrap();
    let probe = BallNoseProbe::new(&mesh_analyzer, probe_radius).unwrap();
    let config = SearchPatternConfig::<DVec3, Vec3>::new(&probe, minimum_z);
    let aabb = Aabb2::with_points(&[DVec2::new(-4.5, -4.50), DVec2::new(5.5, 5.5)]);
    let convex_hull = LineString2::from(aabb);
    let pattern = MeanderPattern::new(aabb, convex_hull, 0.1).unwrap();
    let mut g = c.benchmark_group("edge_tests");
    g.sample_size(10)
        .bench_function("original_triangles_large", |b| {
            b.iter(|| {
                black_box({
                    let _ = pattern.search(&mesh_analyzer, &config).unwrap();
                });
            })
        });
    g.finish();
}

fn split_triangles_large(c: &mut Criterion) {
    let probe_radius: f64 = 1.0;
    let minimum_z = 0.0;
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let filename = Path::new(&manifest_dir).join("benches/sample.obj");
    let mesh_analyzer = MeshAnalyzerBuilder::<DVec3, Vec3>::default()
        .load_from_obj(&filename)
        .unwrap()
        .with_split(probe_radius)
        .unwrap()
        .build()
        .unwrap();
    let probe = BallNoseProbe::new(&mesh_analyzer, probe_radius).unwrap();
    let config = SearchPatternConfig::<DVec3, Vec3>::new(&probe, minimum_z);
    let aabb = Aabb2::with_points(&[DVec2::new(-4.5, -4.50), DVec2::new(5.5, 5.5)]);
    let convex_hull = LineString2::from(aabb);
    let pattern = MeanderPattern::new(aabb, convex_hull, 0.1).unwrap();
    let mut g = c.benchmark_group("edge_tests");
    g.sample_size(10)
        .bench_function("split_triangles_large", |b| {
            b.iter(|| {
                black_box({
                    let _ = pattern.search(&mesh_analyzer, &config).unwrap();
                });
            })
        });
    g.finish();
}

/// test a small radius probe without splitting triangles
fn original_triangles_small(c: &mut Criterion) {
    let probe_radius = 0.5;
    let minimum_z = 0.0;
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let filename = Path::new(&manifest_dir).join("benches/sample.obj");
    let mesh_analyzer = MeshAnalyzerBuilder::<DVec3, Vec3>::default()
        .load_from_obj(&filename)
        .unwrap()
        .build()
        .unwrap();
    let probe = BallNoseProbe::new(&mesh_analyzer, probe_radius).unwrap();
    let config = SearchPatternConfig::<DVec3, Vec3>::new(&probe, minimum_z);
    let aabb = Aabb2::with_points(&[DVec2::new(-4.5, -4.50), DVec2::new(5.5, 5.5)]);
    let convex_hull = LineString2::from(aabb);
    let pattern = MeanderPattern::new(aabb, convex_hull, 0.1).unwrap();
    let mut g = c.benchmark_group("edge_tests");

    g.sample_size(10)
        .bench_function("original_triangles_small", |b| {
            b.iter(|| {
                black_box({
                    let _ = pattern.search(&mesh_analyzer, &config).unwrap();
                });
            })
        });
    g.finish();
}

/// test a small radius probe with splitting triangles
fn split_triangles_small(c: &mut Criterion) {
    let probe_radius = 0.5;
    let minimum_z = 0.0;
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let filename = Path::new(&manifest_dir).join("benches/sample.obj");
    let mesh_analyzer = MeshAnalyzerBuilder::<DVec3, Vec3>::default()
        .load_from_obj(&filename)
        .unwrap()
        .with_split(probe_radius)
        .unwrap()
        .build()
        .unwrap();
    let probe = BallNoseProbe::new(&mesh_analyzer, probe_radius).unwrap();
    let config = SearchPatternConfig::<DVec3, Vec3>::new(&probe, minimum_z);
    let aabb = Aabb2::with_points(&[DVec2::new(-4.5, -4.50), DVec2::new(5.5, 5.5)]);
    let pattern = MeanderPattern::new(aabb, LineString2(aabb.convex_hull().unwrap()), 0.1).unwrap();
    let mut g = c.benchmark_group("edge_tests");
    g.sample_size(10)
        .bench_function("split_triangles_small", |b| {
            b.iter(|| {
                black_box({
                    let _ = pattern.search(&mesh_analyzer, &config).unwrap();
                });
            })
        });
    g.finish();
}

fn bench_edge_detect(c: &mut Criterion) {
    let seed = 3243312434343; // A constant seed for deterministic results.
    let mut rng: StdRng = SeedableRng::seed_from_u64(seed);
    let probe_radius = 2.1;
    let center = DVec2::new(5.0, 5.0);
    let samples: Vec<DVec3> = (0..100)
        .into_iter()
        .map(|_| {
            DVec3::new(
                rng.gen_range(1.0..9.0),
                rng.gen_range(1.0..9.0),
                rng.gen_range(1.0..9.0),
            )
        })
        .collect();
    let mut group = c.benchmark_group("edge_collision_group");
    let mut mt = MaximumTracker::<SearchResult<DVec3>>::default();
    group.bench_function("z_coord_sphere_to_edge_0", |b| {
        b.iter(|| {
            samples.windows(2).for_each(|p| {
                black_box({
                    ball_nose_to_edge_collision::<DVec3>(
                        center,
                        probe_radius,
                        0,
                        p[0],
                        p[1],
                        SkipEndpoint::NoSkip,
                        &mut mt,
                    );
                })
            })
        });
    });
    /*
    group.bench_function("z_coord_sphere_to_edge_1", |b| {
        b.iter(|| {
            samples.windows(2).for_each(|p| {
                black_box({
                    z_coord_sphere_to_edge_1(
                        center,
                        probe_radius,
                        p[0],
                        p[1],
                        SkipEndpoint::NoSkip,
                    );
                })
            })
        });
    });
    group.bench_function("z_coord_sphere_to_edge_2", |b| {
        b.iter(|| {
            samples.windows(2).for_each(|p| {
                black_box({
                    z_coord_sphere_to_edge_2(center, probe_radius, p[0], p[1]);
                })
            })
        });
    });
    group.bench_function("z_coord_sphere_to_edge_3", |b| {
        b.iter(|| {
            samples.windows(2).for_each(|p| {
                black_box({
                    z_coord_sphere_to_edge_3(center, probe_radius, p[0], p[1]);
                })
            })
        });
    });
    group.bench_function("z_coord_sphere_to_edge_4", |b| {
        b.iter(|| {
            samples.windows(2).for_each(|p| {
                black_box({
                    z_coord_sphere_to_edge_4(center, probe_radius, p[0], p[1]);
                })
            })
        });
    });*/
    group.finish();
}

criterion_group!(
    collision_bench,
    split_triangles_large,
    original_triangles_large,
    split_triangles_small,
    original_triangles_small,
    bench_edge_detect,
);

criterion_main!(collision_bench);
