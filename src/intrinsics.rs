use serde::{Deserialize, Serialize};

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
