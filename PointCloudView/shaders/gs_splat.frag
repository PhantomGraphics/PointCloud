#version 450

layout(location = 0) in mat3 vMatrix;
layout(location = 3) in vec4 vColor;

layout(location = 0) out vec4 outColor;

void main() {
    vec2 uv = gl_PointCoord * 2.0 - 1.0;

    float r2 = dot(uv, uv);
    if (r2 > 1.0) {
        discard;
    }

    vec3 g = vMatrix * vec3(uv, 0.0);
    float distSquared = dot(g, g);
    if (distSquared > 1.0) {
        discard;
    }

    float weight = exp(-4.0 * distSquared);
    outColor = vec4(vColor.rgb * weight, vColor.a * weight);
}
