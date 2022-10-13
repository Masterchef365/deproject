use std::io::{BufRead, Write};
use nalgebra::{Point3, Vector3};
use std::path::Path;

use anyhow::{Ok, Result};
use deproject::plane::*;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap();

    let xyz: Vec<Point3<f32>> = load_csv(Path::new(&path))?
        .into_iter()
        .map(Point3::from)
        .collect();

    let plane = ransac_plane(&xyz, 1000, 0.5 / 100.);

    let mut xyzrgb: Vec<[f32; 6]> = vec![];

    for &pt in &xyz {
        let mut v = [0.; 6];
        let pr = plane.proj(pt);
        v[..3].copy_from_slice(pr.coords.as_slice());
        v[1+3] = 1.;
        xyzrgb.push(v);

        let mut v = [0.; 6];
        v[..3].copy_from_slice(pt.coords.as_slice());
        v[0+3] = 1.;
        xyzrgb.push(v);
    }

    write_csv(Path::new("plane_projected.csv"), &xyzrgb)?;

    Ok(())
}

fn load_csv<const N: usize>(path: &Path) -> Result<Vec<[f32; N]>> {
    let s = std::fs::File::open(path)?;
    let s = std::io::BufReader::new(s);

    let mut rows = vec![];

    for line in s.lines() {
        let mut row = [0.0; N];

        line?
            .split(',')
            .zip(row.iter_mut())
            .map(|(v, r)| Ok(*r = v.parse()?))
            .collect::<Result<()>>()?;

        rows.push(row);
    }

    Ok(rows)
}

fn write_csv<const N: usize>(path: &Path, data: &[[f32; N]]) -> Result<()> {
    let s = std::fs::File::create(path)?;
    let mut s = std::io::BufWriter::new(s);

    for row in data {
        for (idx, elem) in row.into_iter().enumerate() {
            if idx != 0 {
                write!(s, ",")?;
            }
            write!(s, "{}", elem)?;
        }

        writeln!(s)?;
    }

    Ok(())
}

