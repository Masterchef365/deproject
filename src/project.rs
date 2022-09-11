use realsense_rust::{base::*, kind::Rs2DistortionModel};

/// Ported from https://github.com/IntelRealSense/librealsense/blob/master/src/rs.cpp
/// Git rev 4e7050a
pub fn rs2_project_point_to_pixel(intrin: Rs2Intrinsics, point: [f32; 3]) -> [f32; 2] {
    let mut x = point[0] / point[2];
    let mut y = point[1] / point[2];

    let distort = intrin.distortion();

    match distort.model {
        Rs2DistortionModel::BrownConradyModified | Rs2DistortionModel::BrownConradyInverse => {
            let r2 = x * x + y * y;
            let f = 1.
                + distort.coeffs[0] * r2
                + distort.coeffs[1] * r2 * r2
                + distort.coeffs[4] * r2 * r2 * r2;
            x *= f;
            y *= f;
            let dx = x + 2. * distort.coeffs[2] * x * y + distort.coeffs[3] * (r2 + 2. * x * x);
            let dy = y + 2. * distort.coeffs[3] * x * y + distort.coeffs[2] * (r2 + 2. * y * y);
            x = dx;
            y = dy;
        }

        Rs2DistortionModel::BrownConrady => {
            let r2 = x * x + y * y;
            let f = 1.
                + distort.coeffs[0] * r2
                + distort.coeffs[1] * r2 * r2
                + distort.coeffs[4] * r2 * r2 * r2;

            let xf = x * f;
            let yf = y * f;

            let dx = xf + 2. * distort.coeffs[2] * x * y + distort.coeffs[3] * (r2 + 2. * x * x);
            let dy = yf + 2. * distort.coeffs[3] * x * y + distort.coeffs[2] * (r2 + 2. * y * y);

            x = dx;
            y = dy;
        }

        Rs2DistortionModel::FThetaFisheye => {
            let mut r = (x * x + y * y).sqrt();
            if r < f32::EPSILON {
                r = f32::EPSILON;
            }
            let rd = 1.0 / distort.coeffs[0] * (2. * r * (distort.coeffs[0] / 2.0).tan()).atan();
            x *= rd / r;
            y *= rd / r;
        }

        Rs2DistortionModel::KannalaBrandt => {
            let mut r = (x * x + y * y).sqrt();
            if r < f32::EPSILON {
                r = f32::EPSILON;
            }
            let theta = r.atan();
            let theta2 = theta * theta;
            let series = 1.
                + theta2
                    * (distort.coeffs[0]
                        + theta2
                            * (distort.coeffs[1]
                                + theta2 * (distort.coeffs[2] + theta2 * distort.coeffs[3])));
            let rd = theta * series;
            x *= rd / r;
            y *= rd / r;
        }

        Rs2DistortionModel::None => (),
    }

    [
        x * intrin.fx() + intrin.ppx(),
        y * intrin.fy() + intrin.ppy(),
    ]
}

pub fn rs2_deproject_pixel_to_point(
    intrin: Rs2Intrinsics,
    pixel: [f32; 2],
    depth: f32,
) -> [f32; 3] {
    //assert(intrin.model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

    let mut x = (pixel[0] - intrin.ppx()) / intrin.fx();
    let mut y = (pixel[1] - intrin.ppy()) / intrin.fy();

    let xo = x;
    let yo = y;

    let distort = intrin.distortion();

    match distort.model {
        Rs2DistortionModel::BrownConradyModified => {
            panic!("Deprojection does not support BrownConradyModified")
        }
        Rs2DistortionModel::BrownConradyInverse => {
            // need to loop until convergence
            // 10 iterations determined empirically
            for _ in 0..10 {
                let r2 = x * x + y * y;
                let icdist = 1.
                    / (1.
                        + ((distort.coeffs[4] * r2 + distort.coeffs[1]) * r2 + distort.coeffs[0])
                            * r2);
                let xq = x / icdist;
                let yq = y / icdist;
                let delta_x =
                    2. * distort.coeffs[2] * xq * yq + distort.coeffs[3] * (r2 + 2. * xq * xq);
                let delta_y =
                    2. * distort.coeffs[3] * xq * yq + distort.coeffs[2] * (r2 + 2. * yq * yq);
                x = (xo - delta_x) * icdist;
                y = (yo - delta_y) * icdist;
            }
        }
        Rs2DistortionModel::BrownConrady => {
            // need to loop until convergence
            // 10 iterations determined empirically
            for _ in 0..10 {
                let r2 = x * x + y * y;
                let icdist = 1.
                    / (1.
                        + ((distort.coeffs[4] * r2 + distort.coeffs[1]) * r2 + distort.coeffs[0])
                            * r2);
                let delta_x =
                    2. * distort.coeffs[2] * x * y + distort.coeffs[3] * (r2 + 2. * x * x);
                let delta_y =
                    2. * distort.coeffs[3] * x * y + distort.coeffs[2] * (r2 + 2. * y * y);
                x = (xo - delta_x) * icdist;
                y = (yo - delta_y) * icdist;
            }
        }
        Rs2DistortionModel::KannalaBrandt => {
            let mut rd = (x * x + y * y).sqrt();
            if rd < f32::EPSILON {
                rd = f32::EPSILON;
            }

            let mut theta = rd;
            let mut theta2 = rd * rd;
            for _ in 0..4 {
                let f = theta
                    * (1.
                        + theta2
                            * (distort.coeffs[0]
                                + theta2
                                    * (distort.coeffs[1]
                                        + theta2
                                            * (distort.coeffs[2] + theta2 * distort.coeffs[3]))))
                    - rd;
                if f.abs() < f32::EPSILON {
                    break;
                }
                let df = 1.
                    + theta2
                        * (3. * distort.coeffs[0]
                            + theta2
                                * (5. * distort.coeffs[1]
                                    + theta2
                                        * (7. * distort.coeffs[2]
                                            + 9. * theta2 * distort.coeffs[3])));
                theta -= f / df;
                theta2 = theta * theta;
            }
            let r = (theta).tan();
            x *= r / rd;
            y *= r / rd;
        }
        Rs2DistortionModel::FThetaFisheye => {
            let mut rd = (x * x + y * y).sqrt();
            if rd < f32::EPSILON {
                rd = f32::EPSILON;
            }
            let r = (distort.coeffs[0] * rd).tan() / (2. * (distort.coeffs[0] / 2.0).tan()).atan();
            x *= r / rd;
            y *= r / rd;
        }
        Rs2DistortionModel::None => (),
    }

    [depth * x, depth * y, depth]
}

pub fn rs2_transform_point_to_point(extrin: Rs2Extrinsics, from_point: [f32; 3]) -> [f32; 3] {
    let rot = extrin.rotation();
    let tl = extrin.translation();
    [
        rot[0] * from_point[0] + rot[3] * from_point[1] + rot[6] * from_point[2] + tl[0],
        rot[1] * from_point[0] + rot[4] * from_point[1] + rot[7] * from_point[2] + tl[1],
        rot[2] * from_point[0] + rot[5] * from_point[1] + rot[8] * from_point[2] + tl[2],
    ]
}
