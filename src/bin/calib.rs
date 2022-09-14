use anyhow::{Context, Result};
use deproject::*;
use std::path::PathBuf;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = PathBuf::from(args.next().context("Missing path arg")?);

    let paths = Paths::from_root(&path)?;

    let v = &paths.horiz[7][9];
    let a = load_color_png(&v[0].color)?;
    let b = load_color_png(&v[1].color)?;

    let a = a.map(|c| [intensity(c)]);
    let b = b.map(|c| [intensity(c)]);

    let d = diff(&a, &b);

    let img = d.map(|c| sgncolor(c[0]));

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

fn intensity(rgb: &[u8]) -> f32 {
    (rgb.iter()
        .map(|&x| u32::from(x))
        .map(|x| x * x)
        .sum::<u32>() as f32)
        .sqrt()
}
