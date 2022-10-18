use anyhow::Result;
use realsense_rust::base::Rs2Intrinsics;
use serde::{Deserialize, Serialize};
use std::str::FromStr;

pub mod image;
pub mod plane;

pub use image::*;

use std::{
    collections::HashSet,
    path::{Path, PathBuf},
};

use clap::Parser;

use crate::project::rs2_deproject_pixel_to_point;
pub mod project;

/// Simple program to greet a person
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
pub struct RecordArgs {
    /// Name of record
    #[clap(short, long, value_parser, default_value = "default")]
    pub name: String,

    /// Maximum stripe granularity
    #[clap(short, long, value_parser, default_value_t = 10)]
    pub max_steps: usize,

    /// Number of samples
    #[clap(short, long, value_parser, default_value_t = 20)]
    pub samples: usize,

    /// Do not fullscreen
    #[clap(short = 'f', long, value_parser)]
    pub no_fullscreen: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct PatternSample {
    /// Step number
    pub step: usize,
    /// Orientation, horizontal if true
    pub orient: bool,
    /// On/Off selection
    pub sign: bool,
    /// Index of this sample
    pub idx: usize,
}

impl ToString for PatternSample {
    fn to_string(&self) -> String {
        format!(
            "{}_{}_{}_{}",
            self.step.to_string(),
            if self.orient { "h" } else { "v" },
            if self.sign { "t" } else { "f" },
            &self.idx.to_string(),
        )
    }
}

impl FromStr for PatternSample {
    type Err = ();
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut parts = s.split('_');

        let step = parts.next().ok_or(())?.parse().map_err(|_| ())?;

        let orient = match parts.next().ok_or(())? {
            "h" => true,
            "v" => false,
            _ => return Err(()),
        };

        let color = match parts.next().ok_or(())? {
            "t" => true,
            "f" => false,
            _ => return Err(()),
        };

        let idx = parts.next().ok_or(())?.parse().map_err(|_| ())?;

        Ok(Self {
            step,
            orient,
            sign: color,
            idx,
        })
    }
}

pub fn record_samples(args: &RecordArgs) -> Vec<Vec<PatternSample>> {
    let mut pat = vec![];
    for orient in [true, false] {
        for step in 0..args.max_steps {
            for color in [true, false] {
                let mut set = vec![];
                for idx in 0..args.samples {
                    set.push(PatternSample {
                        step,
                        orient,
                        sign: color,
                        idx,
                    });
                }
                pat.push(set);
            }
        }
    }
    pat
}

/// /step/idx/[false, true]
pub type SampleSet = Vec<Vec<Vec<SamplePaths>>>;

#[derive(Default, Debug)]
pub struct SamplePaths {
    pub color: PathBuf,
    pub depth: PathBuf,
}

#[derive(Default, Debug)]
pub struct Paths {
    pub vert: SampleSet,
    pub horiz: SampleSet,
}

impl Paths {
    pub fn from_root(path: &Path) -> Result<Paths> {
        let entries: HashSet<PathBuf> = std::fs::read_dir(path)?
            .map(|d| Ok(d?.path()))
            .collect::<Result<HashSet<PathBuf>>>()?;

        let mut paths = Paths::default();

        for orient in [true, false] {
            let set = match orient {
                true => &mut paths.horiz,
                false => &mut paths.vert,
            };

            'step: loop {
                let mut sample_set = vec![];

                'idx: loop {
                    let mut sign_set = vec![];

                    for sign in [true, false] {
                        let sample = PatternSample {
                            step: set.len(),
                            idx: sample_set.len(),
                            orient,
                            sign,
                        };

                        let prefix = sample.to_string();

                        let color = path.join(format!("{prefix}_color.png"));
                        let depth = path.join(format!("{prefix}_depth.png"));

                        if entries.contains(&color) && entries.contains(&depth) {
                            sign_set.push(SamplePaths { color, depth });
                        } else {
                            if sample_set.is_empty() {
                                break 'step;
                            } else {
                                break 'idx;
                            }
                        }
                    }

                    sample_set.push(sign_set);
                }

                set.push(sample_set);
            }
        }

        Ok(paths)
    }
}



#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub struct Rs2IntrinsicsSerde {
    /// Width of the image in pixels"]
    pub width: i32,
    /// Height of the image in pixels"]
    pub height: i32,
    /// Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge"]
    pub ppx: f32,
    /// Vertical coordinate of the principal point of the image, as a pixel offset from the top edge"]
    pub ppy: f32,
    /// Focal length of the image plane, as a multiple of pixel width"]
    pub fx: f32,
    /// Focal length of the image plane, as a multiple of pixel height"]
    pub fy: f32,
    /// Distortion model of the image"]
    pub model: u32,
    /// Distortion coefficients. Order for Brown-Conrady: [k1, k2, p1, p2, k3]. Order for F-Theta Fish-eye: [k1, k2, k3, k4, 0]. Other models are subject to their own interpretations"]
    pub coeffs: [f32; 5usize],
}

impl Into<realsense_sys::rs2_intrinsics> for Rs2IntrinsicsSerde {
    fn into(self) -> realsense_sys::rs2_intrinsics {
        realsense_sys::rs2_intrinsics {
            width: self.width,
            height: self.height,
            ppx: self.ppx,
            ppy: self.ppy,
            fx: self.fx,
            fy: self.fy,
            model: self.model,
            coeffs: self.coeffs,
        }
    }
}

impl From<realsense_sys::rs2_intrinsics> for Rs2IntrinsicsSerde {
    fn from(r: realsense_sys::rs2_intrinsics) -> Self {
        Self {
            width: r.width,
            height: r.height,
            ppx: r.ppx,
            ppy: r.ppy,
            fx: r.fx,
            fy: r.fy,
            model: r.model,
            coeffs: r.coeffs,
        }
    }
}

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
