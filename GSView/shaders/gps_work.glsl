// Prepared once per Gaussian; std430 stride = 112 bytes.
struct Prepared {
    vec4 a; vec4 b; vec4 c; vec4 d;
    vec4 info; // opacity, keep probability, unused, unused
    uvec4 meta; // RNG seed, packed colour, depth key, PBVR flag
    uvec4 count; // sampled count, unused
};
layout(std430, binding = 7) buffer PreparedB { Prepared prepared[]; };
layout(std430, binding = 8) buffer ScanB { uint offsets[]; };
layout(std430, binding = 9) buffer ParticleB { uvec4 particles[]; };
// [0..2] indirect dispatch, [3] total count, [4] 0 cached / 1 replay / 2 primitive fallback.
layout(std430, binding = 10) buffer WorkB { uint work[]; };
