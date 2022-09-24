use anyhow::{Context, Ok, Result};
use deproject::{project::rs2_deproject_pixel_to_point, *};
use nalgebra::{
    DMatrix, DVector, Matrix2x4, Matrix4, Matrix4x2, OMatrix, Point3, Vector2, Vector4, SVD,
};
use rand::prelude::*;
use realsense_rust::base::Rs2Intrinsics;
use std::{
    fs::File,
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
};

type Model = Matrix2x4<f32>;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = PathBuf::from(args.next().context("Missing path arg")?);
    let thresh: f32 = args
        .next()
        .unwrap_or("0.5".to_string())
        .parse::<f32>()
        .context("Threshold must be float")?;

    let paths = Paths::from_root(&path)?;

    let idx = 7;
    let xy = xy_image(&paths, idx)?;

    let color = xy.map(|v| [v[0], v[1], 0.].map(|x| (x.clamp(0., 1.) * 256.) as u8));

    write_color_png("xy.png", &color)?;

    let depth = avg_depth(&path, 100)?;

    let mask = mask(&paths, idx, thresh * 256.)?;
    let mask = mask.zip(&depth, |m, d| [m[0] && d[0] != 0]);

    let mask_color = mask.map(|v| [if v[0] { u8::MAX } else { 0 }; 3]);

    write_color_png("mask.png", &mask_color)?;

    let f = File::open(path.join("intrinsics.json"))?;
    let intrinsics: Rs2IntrinsicsSerde = serde_json::from_reader(f)?;
    let intrinsics: Rs2Intrinsics = Rs2Intrinsics(intrinsics.into());

    //let pcld = pointcloud(&xy, &depth, &mask, &intrinsics);
    let mut pcld = pointcloud(&depth, &mask, &intrinsics);

    let mut pcld_xy: Vec<[f32; 2]> = xy
        .data()
        .chunks_exact(2)
        .zip(mask.data())
        .filter(|(_, mask)| **mask)
        .map(|(xy, _)| [xy[0], xy[1]])
        .collect();

    let rng = SmallRng::seed_from_u64(0);

    let mut output_xyz = pcld.to_vec();
    let mut output_rg = vec![]; //pcld_xy.to_vec();

    //let model = best_model(rng, &pcld, &pcld_xy, 100);
    let model = create_model(&pcld, &pcld_xy).unwrap();

    let mse = model_mse(model, &pcld, &pcld_xy);
    dbg!(mse);

    println!("{}", model);

    for &[x, y, z] in &pcld {
        let xyz = Vector4::new(x, y, z, 1.);
        let uv = model * xyz;
        if uv.x >= 0. && uv.x <= 1. && uv.y >= 0. && uv.y <= 1. {
            output_rg.push([uv.x, uv.y]);
        } else {
            output_rg.push([0.1, 0.1]);
        }
    }

    write_pcld("out.csv", &output_xyz, &output_rg)?;

    //println!("{}", origin);

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

fn model_mse(model: Model, pcld: &[[f32; 3]], xy: &[[f32; 2]]) -> f32 {
    let mut mse = 0.;
    for (&[x, y, z], &[u, v]) in pcld.iter().zip(xy) {
        let uv = Vector2::new(u, v);
        let uv_pred = model * Vector4::new(x, y, z, 1.);

        mse += (uv - uv_pred).norm_squared();
    }

    mse /= pcld.len() as f32;

    mse
}

fn model_origin(model: Model) -> Point3<f32> {
    let mut p = Point3::origin();

    for i in 0..3 {
        p[i] = -model[i * 2 + 1] / model[3 * 2 + 1];
    }

    p
}

fn create_model(pcld: &[[f32; 3]], uv: &[[f32; 2]]) -> Option<Model> {
    /*
    let x = pcld
        .iter()
        .map(|v| [v[0], v[1], v[2], 1.])
        .flatten()
        .collect::<Vec<f32>>();

    let x: DMatrix<f32> = DMatrix::from_row_slice(pcld.len(), 4, &x);

    let u: DMatrix<f32> = DMatrix::from_row_slice(
        pcld.len(),
        2,
        &xy.iter().copied().flatten().collect::<Vec<f32>>(),
    );

    let x =
    */

    // Column-major data
    let mut data = vec![];

    for (pt_xyz, uv) in pcld.iter().zip(uv) {
        for i in 0..2 {
            for k in 0..2 {
                if i == k {
                    data.extend_from_slice(pt_xyz);
                    data.push(1.);
                } else {
                    data.extend_from_slice(&[0.; 4]);
                }
            }

            let xp = -uv[i] / 1.;
            data.extend_from_slice(&pt_xyz.map(|v| v * xp));
            data.push(1. * xp);
        }
    }

    dbg!(data.len());
    dbg!(pcld.len());

    let x = DMatrix::from_row_slice(pcld.len() * 2, 4 * 3, &data);

    let svd = SVD::new(x, false, true);
    let v = svd.v_t.unwrap();
    let vector = v.column(4*3-1);

    println!("{}", svd.singular_values);

    println!("{}", vector);

    let matrix = Model::from_iterator(vector.iter().copied());

    Some(matrix)
}

fn write_pcld(path: impl AsRef<Path>, pcld: &[[f32; 3]], xy: &[[f32; 2]]) -> Result<()> {
    let f = std::fs::File::create(path)?;
    let mut f = BufWriter::new(f);

    for (&[x, y, z], &[u, v]) in pcld.into_iter().zip(xy) {
        writeln!(f, "{},{},{},{},{},0", x, y, z, u, v)?;
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
