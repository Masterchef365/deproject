in vec3 v_pos;
in vec3 v_color;
out vec4 f_color;

void main() {
    vec3 pos = v_pos;
    //pos.xy = pos.xy * 2. - 1.;
    gl_Position = vec4(pos, 1.);
    gl_PointSize = 2.0;
    //vec3 color = vec3(fract(v_color.z * 3.));
    vec3 color = vec3(v_color.xy, 0.);
    f_color = vec4(color, 1.);
}

