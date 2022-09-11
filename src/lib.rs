use clap::Parser;
pub mod project;

/// Simple program to greet a person
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
pub struct RecordArgs {
   /// Name of record
   #[clap(short, long, value_parser)]
   pub name: String,

    /// Maximum stripe granularity
   #[clap(short, long, value_parser, default_value_t = 5)]
   pub max_steps: usize,

   /// Number of samples
   #[clap(short, long, value_parser, default_value_t = 5)]
   pub samples: usize,
}
