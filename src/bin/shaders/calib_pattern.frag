precision mediump float;

in vec2 uv;
out vec4 color;

uniform vec3 params;

float size = params.x;
bool orient = params.y < 1.;
bool sgn = params.z < 1.;

void main() {
    bvec2 f = lessThan(fract(uv * pow(2., size - 1.)), vec2(0.5));

    vec3 c = vec3(((f.x && orient) || (f.y && !orient)) != sgn);

    color = vec4(c, 1.0);
}
