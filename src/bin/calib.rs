use anyhow::{Context, Result};
use deproject::*;
use std::path::PathBuf;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = PathBuf::from(args.next().context("Missing path arg")?);

    let paths = Paths::from_root(&path)?;

    let prep = prepare_data(&paths.horiz, 7)?;

    let d = binary_difftree(&prep);

    let img = d.map(|c| sgncolor(c[0]));

    write_color_png("out.png", &img)?;

    Ok(())
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
