use std::path::PathBuf;

use anyhow::{Context, Result};
use deproject::*;


fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let path = PathBuf::from(args.next().context("Missing path arg")?);

    let paths = Paths::from_root(&path)?;

    dbg!(paths);

    Ok(())
}
