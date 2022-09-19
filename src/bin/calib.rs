use anyhow::{Context, Ok, Result};
use deproject::{project::rs2_deproject_pixel_to_point, *};
use realsense_rust::base::Rs2Intrinsics;
use std::{
    fs::File,
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = PathBuf::from(args.next().context("Missing path arg")?);

    let paths = Paths::from_root(&path)?;

    let idx = 7;
    let xy = xy_image(&paths, idx)?;

    let color = xy.map(|v| [v[0], v[1], 0.].map(|x| (x.clamp(0., 1.) * 256.) as u8));

    write_color_png("xy.png", &color)?;

    let depth = load_depth_png(path.join(format!(
            "{}_depth.png",
            PatternSample {
                step: 0,
                orient: false,
                sign: false,
                idx: 0
            }
            .to_string()
        )))?;

    let mask = mask(&paths, idx, 50.0)?;
    let mask = mask.zip(&depth, |m, d| [m[0] && d[0] != 0]);

    let mask_color = mask.map(|v| [if v[0] { u8::MAX } else { 0 }; 3]);

    write_color_png("mask.png", &mask_color)?;

    let f = File::open(path.join("intrinsics.json"))?;
    let intrinsics: Rs2IntrinsicsSerde = serde_json::from_reader(f)?;
    let intrinsics: Rs2Intrinsics = Rs2Intrinsics(intrinsics.into());

    //let pcld = pointcloud(&xy, &depth, &mask, &intrinsics);
    let pcld = pointcloud(&depth, &mask, &intrinsics);

    write_pcld("out.csv", &pcld)?;

    Ok(())
}

fn write_pcld(path: impl AsRef<Path>, pcld: &[[f32; 3]]) -> Result<()> {
    let f = std::fs::File::create(path)?;
    let mut f = BufWriter::new(f);

    for &[x, y, z] in pcld {
        writeln!(f, "{},{},{}", x, y, z)?;
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
