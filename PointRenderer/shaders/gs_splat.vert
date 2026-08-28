#version 450

layout(location = 0) in vec4 inCenterSize;
layout(location = 1) in vec4 inCovRow0;
layout(location = 2) in vec4 inCovRow1;
layout(location = 3) in vec4 inCovRow2;
layout(location = 4) in vec4 inColor;

layout(binding = 0) uniform UBO {
    mat4 mvp;
} ubo;

layout(location = 0) out mat3 vMatrix;
layout(location = 3) out vec4 vColor;

void main() {
    gl_Position = ubo.mvp * vec4(inCenterSize.xyz, 1.0);
    gl_PointSize = max(inCenterSize.w / max(gl_Position.w, 1.0e-6), 1.0);

    vMatrix = mat3(inCovRow0.xyz, inCovRow1.xyz, inCovRow2.xyz);
    vColor = inColor;
}
