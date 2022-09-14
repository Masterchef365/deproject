use std::{collections::HashSet, path::PathBuf};

use anyhow::{Context, Ok, Result};
use deproject::PatternSample;

/// /step/idx/[false, true]
type SampleSet = Vec<Vec<Vec<SamplePaths>>>;

#[derive(Default, Debug)]
struct SamplePaths {
    color: PathBuf,
    depth: PathBuf,
}

#[derive(Default, Debug)]
struct Paths {
    vert: SampleSet,
    horiz: SampleSet,
}

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = PathBuf::from(args.next().context("Missing path arg")?);

    let entries: HashSet<PathBuf> = std::fs::read_dir(&path)?
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

    dbg!(paths);

    Ok(())
}
