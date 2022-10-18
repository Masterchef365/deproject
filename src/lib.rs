use bytemuck::{Pod, Zeroable};
use realsense_rust::{
    base::Rs2Intrinsics,
    frame::{DepthFrame, PixelKind},
};

pub mod array2d;
pub mod fluid;

pub mod image;
pub mod plane;
pub mod realsense;

pub use image::*;

use crate::realsense::rs2_deproject_pixel_to_point;
pub mod pattern;
pub mod projector;

pub fn pointcloud(
    //xy: &MinimalImage<f32>,
    depth: &Image<u16>,
    mask: &Image<bool>,
    intrinsics: &Rs2Intrinsics,
) -> Vec<[f32; 3]> {
    assert_eq!(mask.width(), depth.width());
    assert_eq!(depth.height(), mask.height());

    let mut pcld = vec![];

    for (row_idx, (depth_row, mask_row)) in depth
        .data()
        .chunks_exact(depth.row_size())
        .zip(mask.data().chunks_exact(mask.row_size()))
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

pub fn pointcloud_fast(
    //xy: &MinimalImage<f32>,
    depth: &DepthFrame,
    intrinsics: &Rs2Intrinsics,
    pcld: &mut Vec<[f32; 3]>,
) {
    pcld.clear();

    let mut data = depth.iter();
    for row_idx in 0..depth.height() {
        for col_idx in 0..depth.width() {
            let depth = match data.next() {
                Some(PixelKind::Z16 { depth }) => *depth,
                _ => unreachable!(),
            };

            if depth == 0 {
                continue;
            }

            let pt = rs2_deproject_pixel_to_point(
                intrinsics,
                [col_idx as f32 - 0.5, row_idx as f32 - 0.5],
                depth as f32,
            );

            let pt = pt.map(|v| v / 1e3);

            pcld.push([pt[0], -pt[1], pt[2]]);
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub color: [f32; 3],
}

unsafe impl Zeroable for Vertex {}
unsafe impl Pod for Vertex {}

impl Vertex {
    pub fn new(pos: [f32; 3], color: [f32; 3]) -> Self {
        Self { pos, color }
    }
}
