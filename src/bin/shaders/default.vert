in vec3 v_pos;
in vec3 v_color;
out vec4 f_color;

void main() {
    vec3 pos = v_pos;
    gl_Position = vec4(pos, 1.);
    gl_PointSize = 2.0;
    vec3 color = v_color;
    f_color = vec4(color, 1.);
}

