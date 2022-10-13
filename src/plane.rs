use std::io::{Write, Read};

use anyhow::Result;
use nalgebra::{Point3, Vector3};
use rand::seq::SliceRandom;

#[derive(Default, Copy, Clone)]
pub struct Plane {
    pub origin: Point3<f32>,
    pub normal: Vector3<f32>,
}

impl Plane {
    pub fn new([origin, a, b]: [Point3<f32>; 3]) -> Self {
        let normal = (a - origin).cross(&(b - origin)).normalize();
        Self { origin, normal }
    }

    pub fn signed_dist(&self, point: Point3<f32>) -> f32 {
        (point - self.origin).dot(&self.normal)
    }

    pub fn distance(&self, point: Point3<f32>) -> f32 {
        self.signed_dist(point).abs()
    }

    pub fn proj(&self, point: Point3<f32>) -> Point3<f32> {
        point - self.signed_dist(point) * self.normal
    }

    pub fn write<W: Write>(&self, mut w: W) -> Result<()> {
        for x in self.origin.coords.as_slice() {
            write!(w, "{},", x)?;
        }

        writeln!(w)?;

        for x in self.normal.as_slice() {
            write!(w, "{},", x)?;
        }

        writeln!(w)?;

        Ok(())
    }

    pub fn read<R: Read>(mut r: R) -> Result<Self> {
        let mut s = String::new();
        r.read_to_string(&mut s)?;

        let mut l = s.lines();

        let mut origin = [0.0; 3];
        l.next().unwrap().split(',').zip(origin.iter_mut()).for_each(|(f, o)| *o = f.parse().unwrap());

        let mut normal = [0.0; 3];
        l.next().unwrap().split(',').zip(normal.iter_mut()).for_each(|(f, n)| *n = f.parse().unwrap());

        Ok(Self {
            origin: Point3::from(origin),
            normal: Vector3::from(normal),
        })

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

pub fn ransac_plane(data: &[Point3<f32>], iters: usize, thresh: f32) -> Plane {
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
