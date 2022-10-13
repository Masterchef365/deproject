use std::io::{BufRead, Write};
use std::path::Path;

use anyhow::{Ok, Result};
use nalgebra::{Point3, Vector3};
use rand::seq::SliceRandom;

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

    write_csv(Path::new("uhhhh.csv"), &xyzrgb)?;

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

#[derive(Default, Copy, Clone)]
struct Plane {
    origin: Point3<f32>,
    normal: Vector3<f32>,
}

impl Plane {
    fn new([origin, a, b]: [Point3<f32>; 3]) -> Self {
        let normal = (a - origin).cross(&(b - origin)).normalize();
        Self { origin, normal }
    }

    fn signed_dist(&self, point: Point3<f32>) -> f32 {
        (point - self.origin).dot(&self.normal)
    }

    fn distance(&self, point: Point3<f32>) -> f32 {
        self.signed_dist(point).abs()
    }

    fn proj(&self, point: Point3<f32>) -> Point3<f32> {
        point - self.signed_dist(point) * self.normal
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plane() {
        let plane = Plane::new([
            Point3::new(0., 0., 1.),
            Point3::new(0., 1., 1.),
            Point3::new(0., 1., 0.3),
        ]);

        assert_eq!(plane.distance(Point3::new(1., 0., 0.)), 1.0);
    }
}

fn ransac_plane(data: &[Point3<f32>], iters: usize, thresh: f32) -> Plane {
    let mut rng = rand::thread_rng();

    let mut best_score = 0;
    let mut best_plane = Plane::default();

    for _ in 0..iters {
        let points = [(); 3].map(|_| *data.choose(&mut rng).unwrap());
        let plane = Plane::new(points);

        let score = data
            .iter()
            .filter(|&&pt| plane.distance(Point3::from(pt)) <= thresh)
            .count();

        if score > best_score {
            best_plane = plane;
            best_score = score;
        }
    }

    best_plane
}
