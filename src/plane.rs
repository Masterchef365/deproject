use std::io::{Read, Write};

use anyhow::Result;
use nalgebra::{ComplexField, Point3, Vector3};
use rand::seq::SliceRandom;

#[derive(Default, Copy, Clone)]
pub struct Plane {
    pub origin: Point3<f32>,
    pub x: Vector3<f32>,
    pub y: Vector3<f32>,
    pub z: Vector3<f32>,
}

impl Plane {
    pub fn new([origin, a, b]: [Point3<f32>; 3]) -> Self {
        let x = (b - origin).normalize();
        let y = (a - origin).cross(&x).normalize();
        let z = x.cross(&y);

        Self { origin, x, y, z }
    }

    pub fn to_planespace(&self, point: Point3<f32>) -> Point3<f32> {
        let v = point - self.origin;
        Point3::new(self.x.dot(&v), self.y.dot(&v), self.z.dot(&v))
    }

    pub fn from_planespace(&self, point: Point3<f32>) -> Point3<f32> {
        let v = self.x * point.x + self.y * point.y + self.z * point.z;
        self.origin + v
    }

    /// Signed distance in the y direction
    pub fn signed_dist(&self, point: Point3<f32>) -> f32 {
        (point - self.origin).dot(&self.y)
    }

    /// Distance in the y direction
    pub fn distance(&self, point: Point3<f32>) -> f32 {
        self.signed_dist(point).abs()
    }

    /// Project a 3D point onto the y dimension of this plane
    pub fn proj(&self, point: Point3<f32>) -> Point3<f32> {
        point - self.signed_dist(point) * self.y
    }

    /// Serialize into the given writer
    pub fn write<W: Write>(&self, mut w: W) -> Result<()> {
        for x in self.origin.coords.as_slice() {
            write!(w, "{},", x)?;
        }

        writeln!(w)?;

        for part in [self.x, self.y, self.z] {
            for x in part.as_slice() {
                write!(w, "{},", x)?;
            }
            writeln!(w)?;
        }

        Ok(())
    }

    /// Deserialize from the given reader
    pub fn read<R: Read>(mut r: R) -> Result<Self> {
        let mut s = String::new();
        r.read_to_string(&mut s)?;

        let mut l = s.lines();

        let mut parse_vect = || {
            let mut part = [0.0; 3];
            l.next()
                .unwrap()
                .split(',')
                .zip(part.iter_mut())
                .for_each(|(f, o)| *o = f.parse().unwrap());
            part
        };

        let origin = Point3::from(parse_vect());
        let x = Vector3::from(parse_vect());
        let y = Vector3::from(parse_vect());
        let z = Vector3::from(parse_vect());

        Ok(Self { origin, x, y, z })
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

pub fn ransac_plane(data: &[Point3<f32>], iters: usize, height: f32, side_len: f32) -> Plane {
    let mut rng = rand::thread_rng();

    let mut best_score = 0;
    let mut best_plane = Plane::default();

    for _ in 0..iters {
        let points = [(); 3].map(|_| *data.choose(&mut rng).unwrap());
        let plane = Plane::new(points);

        let score = data
            .iter()
            .filter(|&&pt| {
                let pt = plane.to_planespace(pt);
                pt.x.abs() < side_len && pt.z.abs() < side_len && pt.y.abs() < height
            })
            .count();

        if score > best_score {
            best_plane = plane;
            best_score = score;
            dbg!(score);
        }
    }

    best_plane
}
