use anyhow::{Context, Result};
use deproject::*;
use std::path::PathBuf;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = PathBuf::from(args.next().context("Missing path arg")?);

    let paths = Paths::from_root(&path)?;

    let v = &paths.horiz[3][9];
    let a = load_color_png(&v[0].color)?;
    let b = load_color_png(&v[1].color)?;

    let a = intensity(&a);
    let b = intensity(&b);

    let d = diff(&a, &b);

    let img = sgncolor_img(&d);

    write_color_png("out.png", &img)?;

    Ok(())
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
    .map(|x| ((x * v.abs() / 256.).clamp(0., 1.) * 256.) as u8)
}

fn sgncolor_img(img: &MinimalImage<f32>) -> MinimalImage<u8> {
    let data = img.data().iter().copied().map(sgncolor).flatten().collect();
    MinimalImage::new(data, img.width(), 3)
}

fn intensity(img: &MinimalImage<u8>) -> MinimalImage<f32> {
    let data = img
        .data()
        .chunks_exact(3)
        .map(|rgb| {
            (rgb.iter()
                .map(|&x| u32::from(x))
                .map(|x| x * x)
                .sum::<u32>() as f32)
                .sqrt()
        })
        .collect();
    MinimalImage {
        data,
        stride: 1,
        row_size: img.width(),
    }
}
