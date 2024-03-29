use anyhow::{ensure, Result};
use bytemuck::{Pod, Zeroable};
use deproject::plane::Plane;
use deproject::{pointcloud, projector::*, Image};
use glow::HasContext;
use glutin::window::Fullscreen;
use nalgebra::Point3;
use std::path::PathBuf;
use std::time::Duration;

use realsense_rust::{
    config::Config,
    context::Context,
    frame::DepthFrame,
    frame::PixelKind,
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
};
use std::fs::File;

fn to_gl_space([x, y, _z]: [f32; 3]) -> [f32; 3] {
    [-(y * 2. - 1.), -(x * 2. - 1.), 0.5]
}

fn main() -> Result<()> {
    // Parse args
    let mut args = std::env::args().skip(1);
    let root_path: PathBuf = args.next().expect("Requires root path").into();

    let model_path = root_path.join("matrix.csv");
    let projector_model = load_projector_model(&model_path)?;

    let plane_path = root_path.join("plane.csv");
    let plane = Plane::read(File::open(plane_path)?)?;

    // Open camera
    // Check for depth or color-compatible devices.
    let context = Context::new()?;
    let devices = context.query_devices(Default::default());
    ensure!(!devices.is_empty(), "No devices found");

    let device = &devices[0];

    // create pipeline
    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();
    config
        .enable_device_from_serial(device.info(Rs2CameraInfo::SerialNumber).unwrap())?
        .disable_all_streams()?
        //.enable_stream(Rs2StreamKind::Color, None, 1280, 0, Rs2Format::Bgr8, 30)?
        .enable_stream(Rs2StreamKind::Depth, None, 1280, 0, Rs2Format::Z16, 30)
        .unwrap();

    // Change pipeline's type from InactivePipeline -> ActivePipeline
    let mut pipeline = pipeline.start(Some(config))?;

    let streams = pipeline.profile().streams();

    let depth_stream = streams
        .iter()
        .find(|p| p.kind() == Rs2StreamKind::Depth)
        .unwrap();
    /*let color_stream = streams
    .iter()
    .find(|p| p.kind() == Rs2StreamKind::Color)
    .unwrap();*/

    let depth_intrinsics = depth_stream.intrinsics()?;
    //let depth_to_color_extrinsics = depth_stream.extrinsics(color_stream)?;
    //let color_intrinsics = color_stream.intrinsics()?;

    //let mut in_color_buf: Vec<[u8; 3]> = vec![];
    //let mut in_depth_buf: Vec<u16> = vec![];
    //let mut out_color_buf: Vec<[u8; 3]> = vec![];

    let total_points = depth_intrinsics.width() * depth_intrinsics.height();

    // Set up opengl
    unsafe {
        let (gl, shader_version, window, event_loop) = {
            let event_loop = glutin::event_loop::EventLoop::new();
            let window_builder = glutin::window::WindowBuilder::new()
                .with_title("Calibrator")
                .with_fullscreen(Some(Fullscreen::Borderless(None)))
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

        let program = gl.create_program().expect("Cannot create program");

        let vertex_shader_source = include_str!("shaders/project.vert");
        let fragment_shader_source = include_str!("shaders/default.frag");

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
        gl.clear_color(0.0, 0.5, 0.5, 1.0);
        gl.enable(glow::VERTEX_PROGRAM_POINT_SIZE);

        // Setup point array
        let point_array = gl.create_vertex_array().unwrap();
        gl.bind_vertex_array(Some(point_array));
        let point_verts = vec![Vertex::default(); total_points];
        let point_buf = gl.create_buffer().expect("Cannot create vertex buffer");
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(point_buf));

        // Set vertex attributes
        gl.enable_vertex_attrib_array(0);
        gl.vertex_attrib_pointer_f32(
            0,
            3,
            glow::FLOAT,
            false,
            std::mem::size_of::<Vertex>() as i32,
            0,
        );

        gl.enable_vertex_attrib_array(1);
        gl.vertex_attrib_pointer_f32(
            1,
            3,
            glow::FLOAT,
            false,
            std::mem::size_of::<Vertex>() as i32,
            3 * std::mem::size_of::<f32>() as i32,
        );

        gl.buffer_data_u8_slice(
            glow::ARRAY_BUFFER,
            bytemuck::cast_slice(&point_verts),
            glow::STREAM_DRAW,
        );

        gl.bind_vertex_array(None);
        gl.bind_buffer(glow::ARRAY_BUFFER, None);

        // Upload projector matrix
        gl.uniform_matrix_4_f32_slice(
            gl.get_uniform_location(program, "u_projector").as_ref(),
            true,
            bytemuck::cast_slice(&projector_model),
        );



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
                    let timeout = Duration::from_millis(2000);
                    let frames = pipeline.wait(Some(timeout)).unwrap();
                    //let color_frame: &ColorFrame = &frames.frames_of_type()[0];
                    let depth_frame: &DepthFrame = &frames.frames_of_type()[0];

                    let in_depth_buf: Vec<u16> = depth_frame
                        .iter()
                        .map(|p| match p {
                            PixelKind::Z16 { depth } => *depth,
                            _ => panic!("{:?}", p),
                        })
                        .collect();

                    let depth_image = Image::new(in_depth_buf, depth_intrinsics.width(), 1);
                    let mask = depth_image.map(|_| [true]);

                    let pointcloud = pointcloud(&depth_image, &mask, &depth_intrinsics);

                    let dist_range = 0.05..0.15;

                    let points: Vec<Vertex> = pointcloud
                        .into_iter()
                        .filter(|p| p[2] != 0.)
                        .filter(|&p| dist_range.contains(&plane.distance(Point3::from(p))))
                        .map(|p| {
                            let mut k = plane.to_planespace(Point3::from(p));
                            k.y = 0.;
                            plane.from_planespace(k)
                        })
                        .map(|p| {
                            Vertex::new(
                                *p.coords.as_ref(),
                                [1., 0., 0.],
                            )
                        })
                        .collect();

                    gl.clear(glow::COLOR_BUFFER_BIT);

                    gl.bind_buffer(glow::ARRAY_BUFFER, Some(point_buf));
                    gl.buffer_data_u8_slice(
                        glow::ARRAY_BUFFER,
                        bytemuck::cast_slice(&points),
                        glow::STREAM_DRAW,
                    );

                    gl.bind_vertex_array(None);
                    gl.bind_buffer(glow::ARRAY_BUFFER, None);

                    gl.bind_vertex_array(Some(point_array));
                    gl.draw_arrays(glow::POINTS, 0, points.len() as _);
                    gl.bind_vertex_array(None);

                    //gl.uniform_3_f32(loc.as_ref(), x, y, z);
                    //gl.draw_arrays(glow::TRIANGLES, 0, 3);

                    window.swap_buffers().unwrap();
                }
                Event::WindowEvent { ref event, .. } => match event {
                    WindowEvent::Resized(physical_size) => {
                        window.resize(*physical_size);
                        gl.viewport(0, 0, physical_size.width as _, physical_size.height as _);
                    }
                    WindowEvent::CloseRequested => {
                        gl.delete_program(program);
                        gl.delete_vertex_array(point_array);
                        *control_flow = ControlFlow::Exit
                    }
                    _ => (),
                },
                _ => (),
            }
        });
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Vertex {
    pub pos: [f32; 3],
    pub color: [f32; 3],
}

unsafe impl Zeroable for Vertex {}
unsafe impl Pod for Vertex {}

impl Vertex {
    pub fn new(pos: [f32; 3], color: [f32; 3]) -> Self {
        Self { pos, color }
    }
}
