#version 450

layout(location = 0) in vec4 fragColor;
layout(location = 0) out vec4 outColor;

void main()
{
	vec2 uv = gl_PointCoord * 2.0 - 1.0;
	float r2 = dot(uv, uv);
	if (r2 > 1.0) {
		discard;
	}

	float gaussian = exp(-2.0 * r2);
	outColor = vec4(fragColor.rgb, fragColor.a * gaussian);
}
