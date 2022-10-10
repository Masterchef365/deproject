use anyhow::{Context, Ok, Result};
use deproject::{project::rs2_deproject_pixel_to_point, *};
use nalgebra::{
    DMatrix, DVector, Matrix2x4, Matrix4, Matrix4x2, OMatrix, Point3, Vector2, Vector4, SVD, Vector3,
};
use rand::prelude::*;
use realsense_rust::base::Rs2Intrinsics;
use std::{
    fs::File,
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let root_path = PathBuf::from(args.next().context("Missing path arg")?);
    let thresh: f32 = args
        .next()
        .unwrap_or("0.5".to_string())
        .parse::<f32>()
        .context("Threshold must be float")?;

    let paths = Paths::from_root(&root_path)?;

    let idx = 7;
    let xy = xy_image(&paths, idx)?;

    let color = xy.map(|v| [v[0], v[1], 0.].map(|x| (x.clamp(0., 1.) * 256.) as u8));

    write_color_png("xy.png", &color)?;

    let depth = avg_depth(&root_path, 100)?;

    let mask = mask(&paths, idx, thresh * 256.)?;
    let mask = mask.zip(&depth, |m, d| [m[0] && d[0] != 0]);

    let mask_color = mask.map(|v| [if v[0] { u8::MAX } else { 0 }; 3]);

    write_color_png("mask.png", &mask_color)?;

    let f = File::open(root_path.join("intrinsics.json"))?;
    let intrinsics: Rs2IntrinsicsSerde = serde_json::from_reader(f)?;
    let intrinsics: Rs2Intrinsics = Rs2Intrinsics(intrinsics.into());

    //let pcld = pointcloud(&xy, &depth, &mask, &intrinsics);
    let pcld = pointcloud(&depth, &mask, &intrinsics);

    let pcld_xy: Vec<[f32; 2]> = xy
        .data()
        .chunks_exact(2)
        .zip(mask.data())
        .filter(|(_, mask)| **mask)
        .map(|(xy, _)| [xy[0], xy[1]])
        .collect();

    let path = root_path.join("calib_points.csv");

    write_pcld(path, &pcld, &pcld_xy)?;

    Ok(())
}

fn avg_depth(path: &Path, max_iters: usize) -> Result<MinimalImage<u16>> {
    let mut accum = load_depth_png(path.join(format!(
            "{}_depth.png",
            PatternSample {
                step: 0,
                orient: false,
                sign: false,
                idx: 0
            }
            .to_string()
        )))?
    .map(|v| [u64::from(v[0])]);

    let mut count = accum.map(|_| [0]);

    let mut n = 0;

    for img in path.read_dir()? {
        let ent = img?;
        let path = ent.path();
        dbg!(&path);
        if path.to_str().unwrap().ends_with("_depth.png") {
            let depth = load_depth_png(path)?;
            accum
                .data_mut()
                .iter_mut()
                .zip(depth.data())
                .for_each(|(a, d)| *a += u64::from(*d));

            count
                .data_mut()
                .iter_mut()
                .zip(depth.data())
                .for_each(|(c, d)| *c += u64::from(*d != 0));

            n += 1;
        }

        if n == max_iters as u64 {
            break;
        }
    }

    let avg = accum.zip(&count, |a, c| {
        if c[0] == 0 {
            [0]
        } else {
            [(a[0] / c[0]) as u16]
        }
    });

    Ok(avg)
}

/*
fn best_model(
    mut rng: impl Rng,
    pcld: &[[f32; 3]],
    xy: &[[f32; 2]],
    iters: usize,
) -> Model {
    let mut best_mse = f32::INFINITY;
    let mut best_model = Matrix2x4::zeros();

    for _ in 0..iters {
        let model = create_model(&mut rng, &pcld, xy).unwrap();
        let mse = model_mse(model, &pcld, xy);

        if mse < best_mse {
            dbg!(mse);
            best_mse = mse;
            best_model = model;
        }

        //output_xyz.push(*origin.coords.as_ref());
        //output_rg.push([mse * 10., 1.]);
    }

    best_model
}
*/

fn model_mse(model: &Model, pcld: &[[f32; 3]], xy: &[[f32; 2]]) -> f32 {
    let mut mse = 0.;
    for (&[x, y, z], &[u, v]) in pcld.iter().zip(xy) {
        let uv = Vector2::new(u, v);
        let uv_pred = f_model(&model, Vector3::new(x, y, z));

        mse += (uv - uv_pred).norm_squared();
    }

    mse /= pcld.len() as f32;

    mse
}

type Model = [DVector<f32>; 2];

fn f_model(model: &Model, pt: Vector3<f32>) -> Vector2<f32> {
    let homo = pt.insert_row(3, 1.);
    let mut xy = [0.; 2];

    for i in 0..2 {
        let a = model[i].transpose().columns(0, 4) * homo;
        let b = model[i].transpose().columns(4, 4) * homo;

        xy[i] = a.as_ref()[0] / b.as_ref()[0];
    }

    Vector2::from(xy)
}

fn create_model(pcld: &[[f32; 3]], uv: &[[f32; 2]]) -> Option<Model> {
    let mut vectors = [DVector::zeros(0), DVector::zeros(0)];

    for uvi in 0..2 {
        // Column-major data
        let mut data = vec![];
        for (xyz, uv) in pcld.iter().zip(uv) {
            let u = uv[uvi];
            let [x, y, z] = *xyz;
            // Homogenous coords
            data.extend_from_slice(&[x, y, z, 1., -u*x, -u*y, -u*z, -u]);
        }

        let x = DMatrix::from_column_slice(8, pcld.len(), &data);

        // SVD in search of null space
        let svd = SVD::new(x.clone(), false, true);
        let v = svd.v_t.unwrap();
        let vector = v.column(v.ncols()-1);

        println!("{}", svd.singular_values);

        println!("{}", vector);

        vectors[uvi] = vector.into_owned();
    }

    Some(vectors)
}

fn write_pcld(path: impl AsRef<Path>, pcld: &[[f32; 3]], xy: &[[f32; 2]]) -> Result<()> {
    let f = std::fs::File::create(path)?;
    let mut f = BufWriter::new(f);

    for (&[x, y, z], &[u, v]) in pcld.into_iter().zip(xy) {
        writeln!(f, "{},{},{},{},{}", x, y, z, u, v)?;
    }

    Ok(())
}

fn pointcloud(
    //xy: &MinimalImage<f32>,
    depth: &MinimalImage<u16>,
    mask: &MinimalImage<bool>,
    intrinsics: &Rs2Intrinsics,
) -> Vec<[f32; 3]> {
    assert_eq!(mask.width(), depth.width());
    assert_eq!(depth.height(), mask.height());

    let mut pcld = vec![];

    for (row_idx, (depth_row, mask_row)) in depth
        .data()
        .chunks_exact(depth.row_size)
        .zip(mask.data().chunks_exact(mask.row_size))
        .enumerate()
    {
        for (col_idx, (depth, mask)) in depth_row.iter().zip(mask_row).enumerate() {
            if *mask {
                let pt = rs2_deproject_pixel_to_point(
                    intrinsics,
                    [col_idx as f32 - 0.5, row_idx as f32 - 0.5],
                    *depth as f32,
                );

                let pt = pt.map(|v| v / 1e3);

                pcld.push([pt[0], -pt[1], pt[2]]);
            }
        }
    }

    pcld
}

fn mask(paths: &Paths, idx: usize, thresh: f32) -> Result<MinimalImage<bool>> {
    let v = 4;
    let sample = &paths.horiz[v][idx];

    let a = load_color_png(&sample[0].color)?.map(|c| [intensity(c)]);
    let b = load_color_png(&sample[1].color)?.map(|c| [intensity(c)]);

    let d = diff(&a, &b);

    let mask = d.map(|v| [v[0].abs() > thresh]);

    Ok(mask)
}

fn xy_image(paths: &Paths, idx: usize) -> Result<MinimalImage<f32>> {
    let prep = prepare_data(&paths.horiz, idx)?;
    let x = binary_difftree(&prep);

    let prep = prepare_data(&paths.vert, idx)?;
    let y = binary_difftree(&prep);

    let xy = x.zip(&y, |x, y| [x[0], y[0]]);

    Ok(xy)
}

fn prepare_data(paths: &SampleSet, idx: usize) -> Result<Vec<MinimalImage<bool>>> {
    let mut levels = vec![];
    for set in &paths[1..] {
        let sample = &set[idx];

        let a = load_color_png(&sample[0].color)?.map(|c| [intensity(c)]);
        let b = load_color_png(&sample[1].color)?.map(|c| [intensity(c)]);

        let d = diff(&a, &b);

        let b = d.map(|c| [c[0] > 0.]);

        levels.push(b);
    }

    Ok(levels)
}

fn binary_difftree(smp: &[MinimalImage<bool>]) -> MinimalImage<f32> {
    let mut img = smp[0].map(|_| [0.]);

    let mut int = 1.;
    for level in smp.iter() {
        int /= 2.;
        img.data_mut()
            .iter_mut()
            .zip(level.data())
            .for_each(|(o, i)| {
                if *i {
                    *o += int
                }
            });
    }

    img
}

fn diff(a: &MinimalImage<f32>, b: &MinimalImage<f32>) -> MinimalImage<f32> {
    let data = a.data().iter().zip(b.data()).map(|(a, b)| a - b).collect();
    MinimalImage {
        data,
        row_size: a.width(),
        stride: 1,
    }
}

fn sgncolor(v: f32) -> [u8; 3] {
    if v > 0. {
        [1., 0.1, 0.1]
    } else {
        [0.1, 0.1, 1.]
    }
    .map(|x| ((x * v.abs()).clamp(0., 1.) * 256.) as u8)
}

fn intensity(rgb: &[u8]) -> f32 {
    (rgb.iter()
        .map(|&x| u32::from(x))
        .map(|x| x * x)
        .sum::<u32>() as f32)
        .sqrt()
}
