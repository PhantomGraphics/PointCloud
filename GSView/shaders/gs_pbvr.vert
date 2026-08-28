#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec4 inColor;   // rgba (a = per-particle alpha)

layout(location = 0) out vec4 fragColor;

layout(binding = 0) uniform UBO {
	mat4 mvp;
	float particleSize;
} ubo;

void main()
{
	// alpha == 0.0 marks GPU-generated padding slots; cull them beyond the far plane.
	if (inColor.a == 0.0) {
		gl_Position  = vec4(0.0, 0.0, 2.0, 1.0);
		gl_PointSize = 0.0;
		fragColor    = vec4(0.0);
		return;
	}

	gl_Position = ubo.mvp * vec4(inPos, 1.0);
	gl_PointSize = ubo.particleSize / gl_Position.w;
	fragColor = inColor;
}
