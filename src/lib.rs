use realsense_rust::base::Rs2Intrinsics;

pub mod image;
pub mod plane;
pub mod intrinsics;

pub use image::*;

use crate::project::rs2_deproject_pixel_to_point;
pub mod project;
pub mod pattern;

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
