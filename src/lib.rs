use anyhow::Result;
use std::fs::File;
use std::io::BufWriter;
use std::str::FromStr;

use std::{
    collections::HashSet,
    path::{Path, PathBuf},
};

use clap::Parser;
pub mod project;

/// Simple program to greet a person
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
pub struct RecordArgs {
    /// Name of record
    #[clap(short, long, value_parser, default_value = "default")]
    pub name: String,

    /// Maximum stripe granularity
    #[clap(short, long, value_parser, default_value_t = 5)]
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

pub fn write_color_png(path: impl AsRef<Path>, image: &MinimalImage<u8>) -> Result<()> {
    let file = File::create(path)?;
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, image.width() as _, image.height() as _); // Width is 2 pixels and height is 1.
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);

    let mut writer = encoder.write_header()?;

    writer.write_image_data(image.data())?;

    Ok(())
}

pub struct MinimalImage<T> {
    pub data: Vec<T>,
    pub row_size: usize,
    pub stride: usize,
}

impl<T> MinimalImage<T> {
    pub fn new(data: Vec<T>, width: usize, stride: usize) -> Self {
        Self {
            data,
            row_size: width * stride,
            stride,
        }
    }

    pub fn height(&self) -> usize {
        self.data.len() / self.row_size
    }

    pub fn width(&self) -> usize {
        self.row_size / self.stride
    }

    pub fn data(&self) -> &[T] {
        &self.data
    }

    pub fn map<U, const N: usize>(&self, f: impl Fn(&[T]) -> [U; N]) -> MinimalImage<U> {
        let data = self
            .data()
            .chunks_exact(self.stride)
            .map(f)
            .flatten()
            .collect();
        MinimalImage::new(data, self.width(), N)
    }
}

pub fn load_color_png(path: impl AsRef<Path>) -> Result<MinimalImage<u8>> {
    let decoder = png::Decoder::new(File::open(path).unwrap());

    let mut reader = decoder.read_info().unwrap();

    let mut data = vec![0; reader.output_buffer_size()];

    let info = reader.next_frame(&mut data).unwrap();

    data.truncate(info.buffer_size());

    Ok(MinimalImage {
        data,
        stride: 3,
        row_size: info.width as usize * 3,
    })
}
