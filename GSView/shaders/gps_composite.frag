#version 450

layout(std430, binding = 0) readonly buffer ResolvedB { uint resolvedBuf[]; };
layout(std140, binding = 1) uniform Params {
    mat4  view;
    vec4  p0;
    vec4  p1;
    uvec4 dims;
    uvec4 ctrl;
    vec4  p2;
    vec4  bg;
} u;

layout(location = 0) out vec4 outColor;

void main() {
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    if (x >= u.dims.x || y >= u.dims.y) {
        outColor = vec4(u.bg.rgb, 1.0);
        return;
    }
    outColor = vec4(unpackUnorm4x8(resolvedBuf[y * u.dims.x + x]).rgb, 1.0);
}
