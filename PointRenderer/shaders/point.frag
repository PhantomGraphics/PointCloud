#version 450

layout(location = 0) in  vec3 fragColor;
layout(location = 0) out vec4 outColor;

void main() {
    // Circular point shape
    vec2 c = gl_PointCoord - vec2(0.5);
    if (dot(c, c) > 0.25) discard;
    outColor = vec4(fragColor, 1.0);
}
