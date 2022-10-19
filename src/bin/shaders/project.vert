uniform mat4 u_projector;

in vec3 v_pos;
in vec3 v_color;
out vec4 f_color;

void main() {
    vec4 p = u_projector * vec4(v_pos, 1.);
    vec2 uv = vec2(p.x/p.y, p.z/p.w);
    uv = -(uv.yx * 2. - 1.);
    gl_Position = vec4(uv, 0.5, 1.);
    gl_PointSize = 2.0;
    f_color = vec4(v_color, 1.);
}

