use anyhow::{Context, Ok, Result};
use deproject::plane::ransac_plane;
use deproject::pointcloud;
use deproject::{image::*, pattern::*, realsense::*};
use nalgebra::Point3;
use rand::prelude::*;
use realsense_rust::base::Rs2Intrinsics;
use std::{
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let root_path = PathBuf::from(args.next().context("Missing path arg")?);
    let snr_thresh: f32 = args
        .next()
        .unwrap_or("2.5".to_string())
        .parse::<f32>()
        .context("Threshold must be float")?;

    let paths = Paths::from_root(&root_path)?;

    let idx = 7;
    let xy = xy_image(&paths, idx)?;

    let color = xy.map(|v| [v[0], v[1], 0.].map(|x| (x.clamp(0., 1.) * 256.) as u8));

    write_color_png("xy.png", &color)?;

    let depth = avg_depth(&root_path, 100)?;

    let mask = mask(&paths, idx, snr_thresh)?;
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

    let xyz: Vec<Point3<f32>> = pcld.iter().copied().map(Point3::from).collect();

    let plane = ransac_plane(&xyz, 1000, 0.5 / 100., 1.0);

    // Write debugging pointcloud
    let path = root_path.join("calib_points.csv");
    write_pcld(path, &pcld, &pcld_xy)?;

    // Write plane location
    let path = root_path.join("plane.csv");
    plane.write(File::create(path)?)?;

    // Calculate projector matrix
    let thresh = 1. / intrinsics.width().max(intrinsics.height()) as f32;
    let iters = 1000;
    let mut rng = rand::thread_rng();
    let model = solve_projector_matrix(&mut rng, &pcld, &pcld_xy, iters, thresh);

    // Write matrix to file
    let mut matrix_csv = File::create(root_path.join("matrix.csv"))?;
    for row in model {
        for elem in row {
            write!(matrix_csv, "{},", elem)?;
        }
        writeln!(matrix_csv)?;
    }

    // Predict model outputs
    let [u_model, v_model] = model;
    let mut u_pred = vec![0f32; pcld.len()];
    let mut v_pred = vec![0f32; pcld.len()];
    predict_row(u_model, &pcld, &mut u_pred);
    predict_row(v_model, &pcld, &mut v_pred);

    // Write predicted points
    let mut predict_csv = BufWriter::new(File::create(root_path.join("predict.csv"))?);
    for i in 0..pcld.len() {
        let [x, y, z] = pcld[i];
        let u = u_pred[i];
        let v = v_pred[i];
        writeln!(predict_csv, "{},{},{},{},{},0.0", x, y, z, u, v)?;
    }

    // Write diff points
    let mut predict_csv = BufWriter::new(File::create(root_path.join("diff.csv"))?);
    for i in 0..pcld.len() {
        let [x, y, z] = pcld[i];
        let u = (u_pred[i] - pcld_xy[i][0]).abs();
        let v = (v_pred[i] - pcld_xy[i][1]).abs();
        writeln!(predict_csv, "{},{},{},{},{},0.0", x, y, z, u, v)?;
    }


    Ok(())
}

fn avg_depth(path: &Path, max_iters: usize) -> Result<Image<u16>> {
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

fn write_pcld(path: impl AsRef<Path>, pcld: &[[f32; 3]], xy: &[[f32; 2]]) -> Result<()> {
    let f = std::fs::File::create(path)?;
    let mut f = BufWriter::new(f);

    for (&[x, y, z], &[u, v]) in pcld.into_iter().zip(xy) {
        writeln!(f, "{},{},{},{},{}", x, y, z, u, v)?;
    }

    Ok(())
}

fn mask(paths: &Paths, idx: usize, snr_thresh: f32) -> Result<Image<bool>> {
    // Calculate average difference per pixel
    let mut diff_sum = load_color_png(&paths.horiz[0][idx][0].color)?.map(|_| [0f32]);
    let mut total = 0.;

    for orient in [&paths.horiz, &paths.vert] {
        for granularity in orient {
            let sample = &granularity[idx];
            let a = load_color_png(&sample[0].color)?.map(|c| [intensity(c)]);
            let b = load_color_png(&sample[1].color)?.map(|c| [intensity(c)]);
            diff_sum
                .data_mut()
                .iter_mut()
                .zip(a.data())
                .zip(b.data())
                .for_each(|((d, a), b)| *d += (*a - *b).abs());
            total += 1.;
        }
    }

    let diff_avg = diff_sum.map(|d| [d[0] / total as f32]);

    // Calculate average deviation from average
    let mut diff_dev_sum = load_color_png(&paths.horiz[0][idx][0].color)?.map(|_| [0f32]);
    for orient in [&paths.horiz, &paths.vert] {
        for granularity in orient {
            let sample = &granularity[idx];
            let a = load_color_png(&sample[0].color)?.map(|c| [intensity(c)]);
            let b = load_color_png(&sample[1].color)?.map(|c| [intensity(c)]);
            diff_dev_sum
                .data_mut()
                .iter_mut()
                .zip(a.data())
                .zip(b.data())
                .zip(diff_avg.data())
                .for_each(|(((d, a), b), avg)| *d += ((*a - *b).abs() - avg).abs());
        }
    }

    let diff_stddev = diff_dev_sum.map(|s| [s[0] / total as f32]);

    let mask = diff_stddev.zip(&diff_avg, |s, a| [a[0] / s[0] > snr_thresh]);

    Ok(mask)
}

fn xy_image(paths: &Paths, idx: usize) -> Result<Image<f32>> {
    let prep = prepare_data(&paths.horiz, idx)?;
    let x = binary_difftree(&prep);

    let prep = prepare_data(&paths.vert, idx)?;
    let y = binary_difftree(&prep);

    let xy = x.zip(&y, |x, y| [x[0], y[0]]);

    Ok(xy)
}

fn prepare_data(paths: &SampleSet, idx: usize) -> Result<Vec<Image<bool>>> {
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

fn binary_difftree(smp: &[Image<bool>]) -> Image<f32> {
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

fn diff(a: &Image<f32>, b: &Image<f32>) -> Image<f32> {
    let data = a.data().iter().zip(b.data()).map(|(a, b)| a - b).collect();
    Image::new(data, a.width(), 1)
}

fn intensity(rgb: &[u8]) -> f32 {
    (rgb.iter()
        .map(|&x| f32::from(x) / 256.0)
        .map(|x| x * x)
        .sum::<f32>()
        / 3.)
        .sqrt()
}

const N: usize = 8;
type ModelRow = [f32; N];
type Model = [ModelRow; 2];

fn solve_projector_matrix(
    mut rng: impl Rng,
    pcld: &[[f32; 3]],
    xy: &[[f32; 2]],
    iters: usize,
    thresh: f32,
) -> Model {
    let (u, v): (Vec<f32>, Vec<f32>) = xy.iter().map(|[x, y]| (x, y)).unzip();

    eprintln!("Solve U:");
    let u_row = solve_row(&mut rng, pcld, &u, iters, thresh);
    eprintln!("Solve V:");
    let v_row = solve_row(&mut rng, pcld, &v, iters, thresh);

    [u_row, v_row]
}

fn solve_row(
    mut rng: impl Rng,
    pcld: &[[f32; 3]],
    u: &[f32],
    iters: usize,
    thresh: f32,
) -> ModelRow {
    let mut best_score = usize::MIN;
    let mut best_model = [0f32; N];

    let mut pred_u = vec![0f32; pcld.len()];
    for _ in 0..iters {
        let model = make_ransac_model(&mut rng, pcld, u);
        predict_row(model, pcld, &mut pred_u);

        let score = calc_score(thresh, u, &pred_u);
        if score > best_score {
            dbg!(score);
            best_score = score;
            best_model = model;
        }
    }

    best_model
}

fn make_ransac_model(mut rng: impl Rng, pcld: &[[f32; 3]], u: &[f32]) -> ModelRow {
    // Sanity check
    assert_eq!(pcld.len(), u.len());

    // Populate matrix randomly
    let mut matrix = [0f32; N * N];
    for row in matrix.chunks_exact_mut(N) {
        let idx = rng.gen_range(0..pcld.len());
        let [x, y, z] = pcld[idx];
        let q = u[idx];

        let xyz1 = [x, y, z, 1.];
        row[..4].copy_from_slice(&xyz1);

        let qxyz1 = xyz1.map(|v| -q * v);
        row[4..].copy_from_slice(&qxyz1);
    }

    // If we used zeros for the guess, the delta would always be zero!
    // So we use one instead
    let mut model = [1f32; N];

    // Magic step size
    let omega = 0.5;

    // Magic step count
    let steps = 150;

    // Use the Modified Richardson Method
    // to determine the matrix coeffs
    for _ in 0..steps {
        let mut next_model = [0f32; N];
        for (i, matrix_row) in matrix.chunks_exact(N).enumerate() {
            let v: f32 = matrix_row.iter().zip(&model).map(|(a, b)| a * b).sum();
            next_model[i] = model[i] - omega * v;
        }
        model = next_model;
    }

    model
}

/// Evaluate model accuracy
fn calc_score(thresh: f32, u: &[f32], pred_u: &[f32]) -> usize {
    u.iter()
        .zip(pred_u)
        .filter_map(|(u, &p)| (p >= 0.0 && p <= 1.0).then(|| u - p))
        .filter(|&v| v.abs() < thresh)
        .count()
}

/// Predict using model
fn predict_row(model: ModelRow, pcld: &[[f32; 3]], output: &mut [f32]) {
    for (xyz, out) in pcld.iter().zip(output) {
        let [x, y, z] = xyz;
        let [a, b, c, d, e, f, g, h] = model;

        let num = a * x + b * y + c * z + d;
        let denom = e * x + f * y + g * z + h;

        *out = num / denom;
    }
}
