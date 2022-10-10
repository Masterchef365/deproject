use std::path::{Path, PathBuf};

use anyhow::{Result, Ok};

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let root_path: PathBuf = args.next().expect("Requires root path").into();
    let calib_points_path = root_path.join("matrix.csv");
    let model = load_model(&calib_points_path)?;

    dbg!(model);

    Ok(())
}

fn load_model(path: &Path) -> Result<[[f32; 8]; 2]> {
    let s = std::fs::read_to_string(path)?;
    let mut v = [[0.0f32; 8]; 2];
    for (line, row) in s.lines().zip(&mut v) {
        line.split(',')
            .zip(row.iter_mut())
            .for_each(|(v, r)| *r = dbg!(v).parse().unwrap());
    }

    Ok(v)
}
