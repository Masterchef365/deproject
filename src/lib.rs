use std::str::FromStr;

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
