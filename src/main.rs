use std::collections::HashSet;
use std::fs::File;
use std::io::BufWriter;
use std::{path::Path, time::Duration};

use glow::*;

use realsense_rust::kind::Rs2Option;
use realsense_rust::{
    config::Config,
    context::Context,
    frame::PixelKind,
    frame::{ColorFrame, DepthFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};

mod project;

use anyhow::{ensure, Ok, Result};

use crate::project::align_images;

fn main() -> Result<()> {
    // Check for depth or color-compatible devices.
    let queried_devices = HashSet::new(); // Query any devices
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    ensure!(!devices.is_empty(), "No devices found");

    let device = &devices[0];

    // create pipeline
    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();
    config
        .enable_device_from_serial(device.info(Rs2CameraInfo::SerialNumber).unwrap())?
        .disable_all_streams()?
        .enable_stream(Rs2StreamKind::Color, None, 640, 0, Rs2Format::Bgr8, 30)?
        .enable_stream(Rs2StreamKind::Depth, None, 0, 240, Rs2Format::Z16, 30)
        .unwrap();

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;

    let streams = pipeline.profile().streams();

    let depth_stream = streams
        .iter()
        .find(|p| p.kind() == Rs2StreamKind::Depth)
        .unwrap();
    let color_stream = streams
        .iter()
        .find(|p| p.kind() == Rs2StreamKind::Color)
        .unwrap();

    let depth_intrinsics = depth_stream.intrinsics()?;
    let depth_to_color_extrinsics = depth_stream.extrinsics(color_stream)?;
    let color_intrinsics = color_stream.intrinsics()?;

    let mut in_color_buf: Vec<[u8; 3]> = vec![];
    let mut in_depth_buf: Vec<u16> = vec![];
    let mut out_color_buf: Vec<[u8; 3]> = vec![];

    let timeout = Duration::from_millis(2000);
    let mut i = 0;
    loop {
        let frames = pipeline.wait(Some(timeout)).unwrap();
        let color_frame: &ColorFrame = &frames.frames_of_type()[0];
        let depth_frame: &DepthFrame = &frames.frames_of_type()[0];

        in_depth_buf.clear();
        in_color_buf.clear();
        out_color_buf.clear();

        in_depth_buf.extend(depth_frame.iter().map(|p| match p {
            PixelKind::Z16 { depth } => depth,
            _ => panic!("{:?}", p),
        }));

        in_color_buf.extend(color_frame.iter().map(|p| match p {
            PixelKind::Bgr8 { b, g, r } => [*r, *g, *b],
            _ => panic!("{:?}", p),
        }));

        out_color_buf.resize(in_depth_buf.len(), [0; 3]);

        align_images(
            &depth_intrinsics,
            &depth_to_color_extrinsics,
            &color_intrinsics,
            &in_depth_buf,
            &in_color_buf,
            &mut out_color_buf,
        );

        let path = format!("images/color_to_depth_{}.png", i);
        write_color_png(
            Path::new(&path),
            depth_frame.width() as _,
            depth_frame.height() as _,
            bytemuck::cast_slice(&out_color_buf),
        )?;

        let path = format!("images/color_{}.png", i);
        write_color_png(
            Path::new(&path),
            color_frame.width() as _,
            color_frame.height() as _,
            bytemuck::cast_slice(&in_color_buf),
        )?;

        let path = format!("images/depth_{}.png", i);
        write_depth_png(
            Path::new(&path),
            depth_frame.width() as u32 * 2,
            depth_frame.height() as _,
            &in_depth_buf,
        )?;

        dbg!(i);

        i += 1;
    }
}

fn write_depth_png(path: &Path, width: u32, height: u32, data: &[u16]) -> Result<()> {
    let file = File::create(path)?;
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width, height); // Width is 2 pixels and height is 1.
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);

    let mut writer = encoder.write_header()?;

    writer.write_image_data(bytemuck::cast_slice(data))?;

    Ok(())
}


fn write_color_png(path: &Path, width: u32, height: u32, data: &[u8]) -> Result<()> {
    let file = File::create(path)?;
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width, height); // Width is 2 pixels and height is 1.
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);

    let mut writer = encoder.write_header()?;

    writer.write_image_data(&data)?;

    Ok(())
}

fn pmain() {
    unsafe {
        let (gl, shader_version, window, event_loop) = {
            let event_loop = glutin::event_loop::EventLoop::new();
            let window_builder = glutin::window::WindowBuilder::new()
                .with_title("Calibrator")
                .with_inner_size(glutin::dpi::LogicalSize::new(1024.0, 768.0));
            let window = glutin::ContextBuilder::new()
                .with_vsync(true)
                .build_windowed(window_builder, &event_loop)
                .unwrap()
                .make_current()
                .unwrap();
            let gl =
                glow::Context::from_loader_function(|s| window.get_proc_address(s) as *const _);
            (gl, "#version 450", window, event_loop)
        };

        let vertex_array = gl
            .create_vertex_array()
            .expect("Cannot create vertex array");
        gl.bind_vertex_array(Some(vertex_array));

        let program = gl.create_program().expect("Cannot create program");

        let vertex_shader_source = include_str!("shaders/fullscreen_tri.vert");
        let fragment_shader_source = include_str!("shaders/calib_pattern.frag");

        let shader_sources = [
            (glow::VERTEX_SHADER, vertex_shader_source),
            (glow::FRAGMENT_SHADER, fragment_shader_source),
        ];

        let mut shaders = Vec::with_capacity(shader_sources.len());

        for (shader_type, shader_source) in shader_sources.iter() {
            let shader = gl
                .create_shader(*shader_type)
                .expect("Cannot create shader");
            gl.shader_source(shader, &format!("{}\n{}", shader_version, shader_source));
            gl.compile_shader(shader);
            if !gl.get_shader_compile_status(shader) {
                panic!("{}", gl.get_shader_info_log(shader));
            }
            gl.attach_shader(program, shader);
            shaders.push(shader);
        }

        gl.link_program(program);
        if !gl.get_program_link_status(program) {
            panic!("{}", gl.get_program_info_log(program));
        }

        for shader in shaders {
            gl.detach_shader(program, shader);
            gl.delete_shader(shader);
        }

        gl.use_program(Some(program));
        gl.clear_color(0.1, 0.2, 0.3, 1.0);

        // We handle events differently between targets

        use glutin::event::{Event, WindowEvent};
        use glutin::event_loop::ControlFlow;

        event_loop.run(move |event, _, control_flow| {
            *control_flow = ControlFlow::Wait;
            match event {
                Event::LoopDestroyed => {
                    return;
                }
                Event::MainEventsCleared => {
                    window.window().request_redraw();
                }
                Event::RedrawRequested(_) => {
                    gl.clear(glow::COLOR_BUFFER_BIT);
                    gl.draw_arrays(glow::TRIANGLES, 0, 3);
                    window.swap_buffers().unwrap();
                }
                Event::WindowEvent { ref event, .. } => match event {
                    WindowEvent::Resized(physical_size) => {
                        window.resize(*physical_size);
                        gl.viewport(0, 0, physical_size.width as _, physical_size.height as _);
                    }
                    WindowEvent::CloseRequested => {
                        gl.delete_program(program);
                        gl.delete_vertex_array(vertex_array);
                        *control_flow = ControlFlow::Exit
                    }
                    _ => (),
                },
                _ => (),
            }
        });
    }
}
