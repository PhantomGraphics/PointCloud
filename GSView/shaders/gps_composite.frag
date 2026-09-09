#version 450

layout(std430, binding = 0) readonly buffer AccumB { vec4 accumBuf[]; };
layout(std140, binding = 1) uniform Params {
    mat4  view;
    vec4  p0;
    vec4  p1;
    uvec4 dims;
    uvec4 ctrl;
    vec4  p2;
    vec4  bg;
    vec4  camPos;
    uvec4 ctrl2;  // resetAccum, shDegree, tonemapMode(0 none/1 Reinhard/2 ACES)
    vec4  p3;     // gamma
} u;

layout(location = 0) out vec4 outColor;

void main() {
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    if (x >= u.dims.x || y >= u.dims.y) {
        outColor = vec4(u.bg.rgb, 1.0);
        return;
    }

    vec4 a = accumBuf[y * u.dims.x + x];
    vec3 c = a.rgb / max(a.a, 1.0);

    uint tm = u.ctrl2.z;
    if (tm == 1u) {
        c = c / (1.0 + c);
    } else if (tm == 2u) {
        c = clamp((c * (2.51 * c + 0.03)) / (c * (2.43 * c + 0.59) + 0.14), 0.0, 1.0);
    }
    c = clamp(c, 0.0, 1.0);

    float g = max(u.p3.x, 0.01);
    c = pow(c, vec3(1.0 / g));

    outColor = vec4(c, 1.0);
}
