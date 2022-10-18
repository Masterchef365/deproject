use ahash::{AHashMap, AHashSet, HashSet, HashSetExt};
use anyhow::Result;
use deproject::array2d::Array2D;
use glow::HasContext;
use glutin::window::Fullscreen;
use nalgebra::{Point2, Point3, Vector2};
use rand::{distributions::Uniform, prelude::Distribution, Rng};
use deproject::Vertex;
use deproject::fluid::{FluidSim, DensitySim};

const MAX_VERTS: usize = 100_000;

fn main() -> Result<()> {
    unsafe {
        let (gl, shader_version, window, event_loop) = {
            let event_loop = glutin::event_loop::EventLoop::new();
            let window_builder = glutin::window::WindowBuilder::new()
                .with_title("Calibrator")
                //.with_fullscreen(Some(Fullscreen::Borderless(None)))
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

        let vertex_shader_source = include_str!("shaders/default.vert");
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
        gl.clear_color(0., 0., 0., 1.0);
        gl.enable(glow::VERTEX_PROGRAM_POINT_SIZE);

        // Setup line array
        let line_array = gl.create_vertex_array().unwrap();
        gl.bind_vertex_array(Some(line_array));
        let mut line_verts = vec![Vertex::default(); MAX_VERTS];
        let line_buf = gl.create_buffer().expect("Cannot create vertex buffer");
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(line_buf));
        line_verts.clear();

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
            bytemuck::cast_slice(&line_verts),
            glow::STREAM_DRAW,
        );

        gl.bind_vertex_array(None);
        gl.bind_buffer(glow::ARRAY_BUFFER, None);

        // We handle events differently between targets

        use glutin::event::{Event, WindowEvent};
        use glutin::event_loop::ControlFlow;

        let mut cursor_pos = (0., 0.);

        let mut tracker = BlobTracker::new(0.01, 20 * 20, i64::MAX);

        let mut points = vec![];

        let sim_size = 150;
        let mut fluid_sim = FluidSim::new(sim_size, sim_size);

        let mut parts = Floaters::new(10_000);

        //let mut density_sim = DensitySim::new(sim_size, sim_size);

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
                    // Create fake pointcloud 
                    points.clear();

                    let (x, y) = cursor_pos;
                    let mut rng = rand::thread_rng();
                    let v = 0.1;
                    let k = 10;
                    for _ in 0..1000 {
                        let dx = fbm(&mut rng, v, k);
                        let dy = fbm(&mut rng, v, k);
                        points.push(Point2::new(x + dx, y + dy));
                    }

                    // Track points
                    tracker.track(&points);

                    // Update fluid
                    delta_to_fluid(fluid_sim.uv_mut(), &tracker);

                    // Step fluid
                    let dt = 0.1;
                    fluid_sim.step(dt, 0.0);

                    // Update particles
                    parts.step(fluid_sim.uv(), dt);

                    // Draw lines
                    line_verts.clear();
                    blob_box_lines(&mut line_verts, &tracker.current);
                    //draw_delta(&mut line_verts, &tracker);
                    //draw_velocity_lines(&mut line_verts, fluid_sim.uv(), 0.5);
                    parts.draw(&mut line_verts);

                    line_verts.truncate(MAX_VERTS);

                    // Render lines
                    gl.clear(glow::COLOR_BUFFER_BIT);

                    gl.bind_buffer(glow::ARRAY_BUFFER, Some(line_buf));
                    gl.buffer_data_u8_slice(
                        glow::ARRAY_BUFFER,
                        bytemuck::cast_slice(&line_verts),
                        glow::STREAM_DRAW,
                    );

                    gl.bind_vertex_array(None);
                    gl.bind_buffer(glow::ARRAY_BUFFER, None);

                    gl.bind_vertex_array(Some(line_array));
                    gl.draw_arrays(glow::LINES, 0, line_verts.len() as _);
                    gl.bind_vertex_array(None);

                    //gl.uniform_3_f32(loc.as_ref(), x, y, z);
                    //gl.draw_arrays(glow::TRIANGLES, 0, 3);

                    window.swap_buffers().unwrap();
                }
                Event::WindowEvent { ref event, .. } => match event {
                    WindowEvent::CursorMoved { position, .. } => {
                        let ph = window.window().inner_size();
                        cursor_pos = (
                            2. * position.x as f32 / ph.width as f32 - 1.,
                            -2. * position.y as f32 / ph.height as f32 + 1.,
                        );
                    }
                    WindowEvent::Resized(physical_size) => {
                        window.resize(*physical_size);
                        gl.viewport(0, 0, physical_size.width as _, physical_size.height as _);
                    }
                    WindowEvent::CloseRequested => {
                        gl.delete_program(program);
                        gl.delete_vertex_array(line_array);
                        *control_flow = ControlFlow::Exit
                    }
                    _ => (),
                },
                _ => (),
            }
        });
    }
}


fn fbm(mut rng: impl Rng, a: f32, iters: usize) -> f32 {
    let s = Uniform::new(-a, a);
    let mut out = 0.0;
    for _ in 0..iters {
        out /= 2.;
        out += s.sample(&mut rng);
    }
    out
}

struct BlobTracker {
    current: BlobBoxes,
    last: BlobBoxes,
    cell_width: f32,
    delta: AHashMap<Point2<i32>, Vector2<f32>>,
}

impl BlobTracker {
    pub fn new(cell_width: f32, min_blob_area: i64, max_blob_area: i64) -> Self {
        let bb = || BlobBoxes::new(cell_width, min_blob_area, max_blob_area);
        Self {
            cell_width,
            current: bb(),
            last: bb(),
            delta: Default::default(),
        }
    }

    pub fn delta(&self) -> &AHashMap<Point2<i32>, Vector2<f32>> {
        &self.delta
    }

    /// Returns (position, vector delta) for each point that should move
    pub fn track(&mut self, points: &[Point2<f32>]) {
        // -> Vec<(Point2<i32>, Vector2<f32>)> {
        std::mem::swap(&mut self.current, &mut self.last);
        self.current.insert(points);
        self.compute_delta();
    }

    fn compute_delta(&mut self) {
        self.delta.clear();
        for last in &self.last.bounds {
            for current in &self.current.bounds {
                if last.intersects(&current) {
                    let diff = current.center() - last.center();
                    let diff = diff.cast::<f32>() * self.cell_width;

                    for rect in [last, current] {
                        for x in rect.min.x..rect.max.x {
                            for y in rect.min.y..rect.max.y {
                                let pt = Point2::new(x, y);
                                self.delta.insert(pt, diff);
                            }
                        }
                    }
                }
            }
        }
    }
}

struct BlobBoxes {
    cell_width: f32,
    min_blob_area: i64,
    max_blob_area: i64,
    boxes: AHashMap<Point2<i32>, usize>,
    tmp: AHashMap<Point2<i32>, usize>,
    bounds: Vec<Rect2>,
}

impl BlobBoxes {
    pub fn new(cell_width: f32, min_blob_area: i64, max_blob_area: i64) -> Self {
        Self {
            cell_width,
            min_blob_area,
            max_blob_area,
            boxes: Default::default(),
            tmp: Default::default(),
            bounds: Default::default(),
        }
    }

    pub fn insert(&mut self, points: &[Point2<f32>]) {
        self.clear();
        self.quantize(points);
        self.find_blobs();
    }

    fn clear(&mut self) {
        self.boxes.clear();
        self.bounds.clear();
        self.tmp.clear();
    }

    fn quantize(&mut self, points: &[Point2<f32>]) {
        for point in points {
            let key = point / self.cell_width;
            let key = Point2::new(key.x as i32, key.y as i32);
            *self.boxes.entry(key).or_insert(0) += 1;
        }
    }

    fn find_blobs(&mut self) {
        let mut search_buf = vec![];
        self.tmp.clear();
        self.tmp.extend(self.boxes.iter());
        self.bounds.clear();

        while let Some(&seed) = self.tmp.keys().next() {
            let (seed, _count) = self.tmp.remove_entry(&seed).unwrap();
            search_buf.push(seed);
            let mut bbox = Rect2::new(seed);

            while let Some(part) = search_buf.pop() {
                let offsets = [
                    Vector2::new(1, 0),
                    Vector2::new(-1, 0),
                    Vector2::new(0, 1),
                    Vector2::new(0, -1),
                ];

                for off in offsets {
                    let subject = part + off;
                    if let Some((s, _)) = self.tmp.remove_entry(&subject) {
                        search_buf.push(s);
                        bbox.insert(s);
                    }
                }
            }

            if (self.min_blob_area..=self.max_blob_area).contains(&bbox.area()) {
                self.bounds.push(bbox);
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct Rect2 {
    min: Point2<i32>,
    max: Point2<i32>,
}

impl Rect2 {
    pub fn new(pt: Point2<i32>) -> Self {
        Self { min: pt, max: pt }
    }

    pub fn insert(&mut self, pt: Point2<i32>) {
        self.min.x = self.min.x.min(pt.x);
        self.min.y = self.min.y.min(pt.y);

        self.max.x = self.max.x.max(pt.x);
        self.max.y = self.max.y.max(pt.y);
    }

    pub fn area(&self) -> i64 {
        let s = self.max - self.min;
        let s = s.cast::<i64>();
        s.x * s.y
    }

    pub fn center(&self) -> Point2<i32> {
        self.min + (self.max - self.min) / 2
    }

    pub fn intersects(&self, other: &Self) -> bool {
        !(self.max.x < other.min.x
            || other.max.x < self.min.x
            || self.max.y < other.min.y
            || other.max.y < self.min.y)
    }
}

fn blob_box_lines(lines: &mut Vec<Vertex>, blob_boxes: &BlobBoxes) {
    let mut push_vert = |pt: Point2<f32>, color: [f32; 3]| {
        lines.push(Vertex {
            pos: [pt.x, pt.y, 0.5],
            color,
        })
    };

    let mut draw_box = |min: Point2<i32>, max: Point2<i32>, color: [f32; 3]| {
        let bw = blob_boxes.cell_width;
        let tl: Point2<f32> = min.cast() * bw;
        let br: Point2<f32> = max.cast() * bw + Vector2::from_element(bw);

        let tr = Point2::new(br.x, tl.y);
        let bl = Point2::new(tl.x, br.y);

        push_vert(tl, color);
        push_vert(bl, color);

        push_vert(tl, color);
        push_vert(tr, color);

        push_vert(tr, color);
        push_vert(br, color);

        push_vert(bl, color);
        push_vert(br, color);
    };

    for (&tl, _) in &blob_boxes.boxes {
        draw_box(tl, tl, [1., 0., 0.]);
    }

    for rect in &blob_boxes.bounds {
        draw_box(rect.min, rect.max, [1., 1., 0.]);
    }
}

fn draw_delta(lines: &mut Vec<Vertex>, tracker: &BlobTracker) {
    let mut push_vertex = |pt: Point2<f32>, color: [f32; 3]| {
        lines.push(Vertex {
            pos: [pt.x, pt.y, 0.5],
            color,
        })
    };

    for (pt, v) in tracker.delta() {
        let pt = pt.cast::<f32>() * tracker.cell_width;
        push_vertex(pt, [0.; 3]);
        push_vertex(pt + *v, [1.; 3]);
    }
}

fn draw_velocity_lines(lines: &mut Vec<Vertex>, (u, v): (&Array2D<f32>, &Array2D<f32>), z: f32) {
    let mut push_vertex = |pt: Point2<f32>, color: [f32; 3]| {
        lines.push(Vertex {
            pos: [pt.x, pt.y, 0.5],
            color,
        })
    };

    let cell_width = 2. / u.width() as f32;
    let cell_height = 2. / u.height() as f32;

    for i in 0..u.width() {
        let i_frac = (i as f32 / u.width() as f32) * 2. - 1.;
        for j in 0..u.height() {
            let j_frac = (j as f32 / u.height() as f32) * 2. - 1.;

            let u = u[(i, j)];
            let v = v[(i, j)];

            let speed = (u.powf(2.) + v.powf(2.)).sqrt();

            let color = [speed * 80.; 3];

            let tail = Point2::new(
                i_frac + cell_width / 2.,
                j_frac + cell_height / 2.,
            );
            push_vertex(tail, [0.; 3]);

            let len = cell_height * 2. / speed;
            let tip = tail + Vector2::new(u, v) * len;
            push_vertex(tip, color);
        }
    }
}

fn delta_to_fluid((u, v): (&mut Array2D<f32>, &mut Array2D<f32>), tracker: &BlobTracker) {
    let w = u.width() as f32;
    let h = u.height() as f32;

    for (pt, vt) in tracker.delta() {
        let pt = (pt.cast::<f32>() * tracker.cell_width) / 2. + Vector2::from_element(0.5);
        let i = (pt.x * w).clamp(0., w-1.) as usize;
        let j = (pt.y * h).clamp(0., h-1.) as usize;

        u[(i, j)] += vt.x;
        v[(i, j)] += vt.y;
    }
}

struct Floaters {
    parts: Vec<Point2<f32>>,
    last: Vec<Point2<f32>>,
    mask: Vec<bool>,
}

impl Floaters {
    pub fn new(n: usize) -> Self {
        let parts = vec![Point2::new(-10., -10.); n];
        let last = parts.clone();
        let mask = vec![false; n];

        Self { parts, last, mask }
    }

    pub fn step(&mut self, (u, v): (&Array2D<f32>, &Array2D<f32>), dt: f32) {
        std::mem::swap(&mut self.parts, &mut self.last);

        let mut rng = rand::thread_rng();
        let unif = Uniform::new(-1., 1.);

        let kill_rate = 0.0001;

        let w = u.width() as f32;
        let h = u.height() as f32;
        for ((part, last), mask) in self.parts.iter_mut().zip(&self.last).zip(&mut self.mask) {
            let in_bounds = part.x >= -1. && part.x <= 1. && part.y >= -1. && part.y <= 1.;

            let alive = in_bounds && rng.gen_bool(1. - kill_rate);

            if alive {
                let pt = part.cast::<f32>() / 2. + Vector2::from_element(0.5);
                let i = (pt.x * w).clamp(0., w-1.) as usize;
                let j = (pt.y * h).clamp(0., h-1.) as usize;
                let uv = Vector2::new(u[(i,j)], v[(i,j)]);
                *part = last + uv * dt;
            } else {
                *part = Point2::new(
                    unif.sample(&mut rng),
                    unif.sample(&mut rng),
                );
            }

            *mask = alive;
        }

    }

    pub fn draw(&self, lines: &mut Vec<Vertex>) {
        let mut push_vertex = |pt: Point2<f32>, color: [f32; 3]| {
            lines.push(Vertex {
                pos: [pt.x, pt.y, 0.5],
                color,
            })
        };


        for ((part, last), mask) in self.parts.iter().zip(&self.last).zip(&self.mask) {
            if *mask {
                push_vertex(*last, [1.; 3]);
                push_vertex(*part, [1.; 3]);
            }
        }
    }
}
