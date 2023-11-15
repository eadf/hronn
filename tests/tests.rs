use hronn::prelude::*;

#[test]
fn test_collision_ball_glam_f64() -> Result<(), HronnError> {
    use hronn::prelude::*;
    use std::{env, path::Path};
    use vector_traits::glam::{DVec3, Vec3};

    //let relative_filename = "flat_edge.obj",
    //let relative_filename = "sample.obj",
    //let relative_filename = "dual_test.obj",
    //let relative_filename = "sample2.obj",
    //let relative_filename = "simple_triangle.obj",
    let test_data_filename = "cube.obj";
    let bounding_shape_filename = "boundingshape.obj";
    let result_filename = "toolpath.obj";
    let probe_radius = 0.5;
    let minimum_z = 0.0;
    let step = 0.3699999749660492;

    type ProcessingType = DVec3;
    type MeshType = Vec3;

    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let filename = Path::new(&manifest_dir)
        .join("test_data")
        .join(test_data_filename);
    let bs_filename = Path::new(&manifest_dir)
        .join("test_data")
        .join(bounding_shape_filename);
    let result_filename = Path::new(&manifest_dir).join(result_filename);
    let mesh_analyzer = MeshAnalyzerBuilder::<ProcessingType, MeshType>::default()
        .load_from_obj(&filename)?
        //.with_split(probe_radius)?
        .build()?;
    //let mesh_analyzer = MeshAnalyzerBuilder::new()
    //    .load_from_obj(&filename)?
    //    .build()?;
    let probe = BallNoseProbe::<ProcessingType, MeshType>::new(&mesh_analyzer, probe_radius)?;
    //let probe = SquareEndProbe::new(&mesh_analyzer, probe_radius)?;
    //let pattern = AabbMeanderPattern::new(minimum_z, bb, step)?.filename(toolpath)?;
    //let pattern = TriangulatePattern::new(minimum_z, bb, step)?.filename(toolpath)?;
    let pattern = {
        let bs = Obj::<MeshType>::new_from_file(&bs_filename)?;
        //let vertices:Vec<MeshType> = bs.vertices.into_iter().map(|v|v.into()).collect();
        let (aabb, hull) = generate_convex_hull_then_aabb(&bs.vertices)?;
        MeanderPattern::<ProcessingType, MeshType>::new(aabb, hull, step)?
    };
    let config = SearchPatternConfig::<ProcessingType, MeshType>::new(&probe, minimum_z);
    match pattern.search(&mesh_analyzer, &config)? {
        StrategyResult::MeshData(mesh) => {
            if false {
                /*for samples in mesh.indices.windows(2) {
                    let p0: ProcessingType = mesh.vertices[samples[0]].into();
                    let p0 = p0.to_2d();
                    let p1: ProcessingType = mesh.vertices[samples[1]].into();
                    let p1 = p1.to_2d();
                    assert!(
                        p0.distance(p1) < step * 2.10,
                        "p0:{} p1:{} distance:{} step:{}",
                        p0,
                        p1,
                        p0.distance(p1),
                        step
                    );
                }*/
                mesh.save_as_obj(&result_filename, "mesh")?;
            }
        }
        StrategyResult::LineData(line) => {
            if false {
                /*
                for samples in line.lines.first().unwrap().windows(2) {
                    let p0: ProcessingType = line.vertices[samples[0]].into();
                    let p0 = p0.to_2d();
                    let p1: ProcessingType = line.vertices[samples[1]].into();
                    let p1 = p1.to_2d();
                    assert!(
                        p0.distance(p1) < step * 2.10,
                        "p0:{} p1:{} distance:{} step:{}",
                        p0,
                        p1,
                        p0.distance(p1),
                        step
                    );
                }*/
                line.save_as_obj(&result_filename, "line")?;
            }
        }
    }
    Ok(())
}

#[test]
fn test_collision_ball_glam_f32() -> Result<(), HronnError> {
    use hronn::prelude::*;
    use std::{env, path::Path};
    use vector_traits::glam::{Vec3, Vec3A};

    //let relative_filename = "flat_edge.obj",
    //let relative_filename = "sample.obj",
    //let relative_filename = "dual_test.obj",
    //let relative_filename = "sample2.obj",
    //let relative_filename = "simple_triangle.obj",
    let test_data_filename = "cube.obj";
    let bounding_shape_filename = "boundingshape.obj";
    //let result_filename = "toolpath.obj";
    let probe_radius = 0.5;
    let minimum_z = 0.0;
    let step = 0.3699999749660492;

    type ProcessingType = Vec3A;
    type MeshType = Vec3;

    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let filename = Path::new(&manifest_dir)
        .join("test_data")
        .join(test_data_filename);
    let bs_filename = Path::new(&manifest_dir)
        .join("test_data")
        .join(bounding_shape_filename);
    //let result_filename = Path::new(&manifest_dir).join(result_filename);
    let mesh_analyzer = MeshAnalyzerBuilder::<ProcessingType, MeshType>::default()
        .load_from_obj(&filename)?
        //.with_split(probe_radius)?
        .build()?;
    //let mesh_analyzer = MeshAnalyzerBuilder::new()
    //    .load_from_obj(&filename)?
    //    .build()?;
    let probe = BallNoseProbe::<ProcessingType, MeshType>::new(&mesh_analyzer, probe_radius)?;
    //let probe = SquareEndProbe::new(&mesh_analyzer, probe_radius)?;
    //let pattern = AabbMeanderPattern::new(minimum_z, bb, step)?.filename(toolpath)?;
    //let pattern = TriangulatePattern::new(minimum_z, bb, step)?.filename(toolpath)?;
    let pattern = {
        let bs = Obj::<MeshType>::new_from_file(&bs_filename)?;
        //let vertices:Vec<MeshType> = bs.vertices.into_iter().map(|v|v.into()).collect();
        let (aabb, hull) = generate_convex_hull_then_aabb(&bs.vertices)?;
        MeanderPattern::<ProcessingType, MeshType>::new(aabb, hull, step)?
    };
    let config = SearchPatternConfig::<ProcessingType, MeshType>::new(&probe, minimum_z);
    let _ = pattern.search(&mesh_analyzer, &config)?;
    Ok(())
}

#[test]
fn test_collision_se_glam_f32() -> Result<(), HronnError> {
    use hronn::prelude::*;
    use std::{env, path::Path};
    use vector_traits::glam::Vec3;

    //let relative_filename = "flat_edge.obj",
    //let relative_filename = "sample.obj",
    //let relative_filename = "dual_test.obj",
    //let relative_filename = "sample2.obj",
    //let relative_filename = "simple_triangle.obj",
    let test_data_filename = "cube.obj";
    let bounding_shape_filename = "boundingshape.obj";
    //let result_filename = "toolpath.obj";
    let probe_radius = 0.5;
    let minimum_z = 0.0;
    let step = 0.3699999749660492;

    type ProcessingType = Vec3;
    type MeshType = Vec3;

    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let filename = Path::new(&manifest_dir)
        .join("test_data")
        .join(test_data_filename);
    let bs_filename = Path::new(&manifest_dir)
        .join("test_data")
        .join(bounding_shape_filename);
    //let result_filename = Path::new(&manifest_dir).join(result_filename);
    let mesh_analyzer = MeshAnalyzerBuilder::<ProcessingType, MeshType>::default()
        .load_from_obj(&filename)?
        //.with_split(probe_radius)?
        .build()?;
    //let mesh_analyzer = MeshAnalyzerBuilder::new()
    //    .load_from_obj(&filename)?
    //    .build()?;
    //let probe = BallNoseProbe::<ProcessingType, MeshType>::new(&mesh_analyzer, probe_radius)?;
    let probe = SquareEndProbe::new(&mesh_analyzer, probe_radius)?;
    //let pattern = AabbMeanderPattern::new(minimum_z, bb, step)?.filename(toolpath)?;
    //let pattern = TriangulatePattern::new(minimum_z, bb, step)?.filename(toolpath)?;
    let pattern = {
        let bs = Obj::<MeshType>::new_from_file(&bs_filename)?;
        //let vertices:Vec<MeshType> = bs.vertices.into_iter().map(|v|v.into()).collect();
        let (aabb, hull) = generate_convex_hull_then_aabb(&bs.vertices)?;
        MeanderPattern::<ProcessingType, MeshType>::new(aabb, hull, step)?
    };
    let config = SearchPatternConfig::<ProcessingType, MeshType>::new(&probe, minimum_z);
    let _ = pattern.search(&mesh_analyzer, &config)?;
    Ok(())
}

#[test]
fn test_collision_se_tri_glam_f32() -> Result<(), HronnError> {
    use hronn::prelude::*;
    use std::{env, path::Path};
    use vector_traits::glam::Vec3;

    //let relative_filename = "flat_edge.obj",
    //let relative_filename = "sample.obj",
    //let relative_filename = "dual_test.obj",
    //let relative_filename = "sample2.obj",
    //let relative_filename = "simple_triangle.obj",
    let test_data_filename = "cube.obj";
    let bounding_shape_filename = "boundingshape.obj";
    //let result_filename = "toolpath.obj";
    let probe_radius = 0.5;
    let minimum_z = 0.0;
    let step = 0.3699999749660492;

    type ProcessingType = Vec3;
    type MeshType = Vec3;

    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let filename = Path::new(&manifest_dir)
        .join("test_data")
        .join(test_data_filename);
    let bs_filename = Path::new(&manifest_dir)
        .join("test_data")
        .join(bounding_shape_filename);
    //let result_filename = Path::new(&manifest_dir).join(result_filename);
    let mesh_analyzer = MeshAnalyzerBuilder::<ProcessingType, MeshType>::default()
        .load_from_obj(&filename)?
        //.with_split(probe_radius)?
        .build()?;
    //let mesh_analyzer = MeshAnalyzerBuilder::new()
    //    .load_from_obj(&filename)?
    //    .build()?;
    //let probe = BallNoseProbe::<ProcessingType, MeshType>::new(&mesh_analyzer, probe_radius)?;
    let probe = SquareEndProbe::new(&mesh_analyzer, probe_radius)?;
    let pattern = {
        let bs = Obj::<MeshType>::new_from_file(&bs_filename)?;
        //let vertices:Vec<MeshType> = bs.vertices.into_iter().map(|v|v.into()).collect();
        let (aabb, hull) = generate_convex_hull_then_aabb(&bs.vertices)?;
        TriangulatePattern::<ProcessingType, MeshType>::new(aabb, hull, step)?
    };
    let config = SearchPatternConfig::<ProcessingType, MeshType>::new(&probe, minimum_z);
    let _ = pattern.search(&mesh_analyzer, &config)?;
    Ok(())
}
