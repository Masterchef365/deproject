use anyhow::Result;
use std::path::Path;

pub type ProjectorModel = [[f32; 8]; 2];

pub fn deproject(model: &ProjectorModel, [x, y, z]: [f32; 3]) -> [f32; 3] {
    let model_u = model[0];
    let model_v = model[1];

    let w_u = model_u[4] * x + model_u[5] * y + model_u[6] * z + model_u[7];
    let w_v = model_v[4] * x + model_v[5] * y + model_v[6] * z + model_v[7];

    let u = model_u[0] * x + model_u[1] * y + model_u[2] * z + model_u[3];
    let v = model_v[0] * x + model_v[1] * y + model_v[2] * z + model_v[3];

    [u / w_u, v / w_v, w_u]
}

pub fn load_projector_model(path: &Path) -> Result<ProjectorModel> {
    let s = std::fs::read_to_string(path)?;
    let mut v = [[0.0f32; 8]; 2];
    for (line, row) in s.lines().zip(&mut v) {
        line.split(',')
            .zip(row.iter_mut())
            .for_each(|(v, r)| *r = v.parse().unwrap());
    }

    Ok(v)
}
