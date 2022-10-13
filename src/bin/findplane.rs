use std::io::BufRead;
use std::path::Path;

use anyhow::{Ok, Result};
use nalgebra::{Point3, Vector3};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args.next().unwrap();
    //let xyz: Vec<[f32; 3]> = load_csv(Path::new(&path))?;

    //dbg!(xyz);

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

struct Plane {
    origin: Point3<f32>,
    normal: Vector3<f32>,
}

impl Plane {
    fn new([origin, a, b]: [Point3<f32>; 3]) -> Self {
        let normal = (a - origin).cross(&(b - origin)).normalize();
        Self { origin, normal }
    }

    fn distance(&self, point: Point3<f32>) -> f32 {
        (point - self.origin).dot(&self.normal).abs()
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
