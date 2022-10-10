in vec3 v_pos;
in vec3 v_color;
out vec4 f_color;

void main() {
    vec3 pos = v_pos;
    f_color = vec4(pos, 1.);
}

