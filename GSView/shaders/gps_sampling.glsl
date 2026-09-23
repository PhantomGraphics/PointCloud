const float PI2_6  = 1.6449340668482264;
const float TWO_PI = 6.283185307179586;
const float SH_C1  = 0.4886025119029199;

uint pcg(uint v) {
    uint s = v * 747796405u + 2891336453u;
    uint w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
    return (w >> 22u) ^ w;
}
struct RS { uint s; };
float rnd(inout RS r) { r.s = pcg(r.s); return float(r.s) * (1.0 / 4294967296.0); }

uint packLinearColor(vec3 color) {
    vec3 encoded = clamp(log2(max(color, vec3(0.0)) + vec3(1.0)) / 16.0,
                         vec3(0.0), vec3(1.0));
    uvec3 quantized = uvec3(round(encoded * vec3(2047.0, 2047.0, 1023.0)));
    return quantized.x | (quantized.y << 11u) | (quantized.z << 22u);
}

float dilogSeries(float x) {
    float t = x, s = x;
    for (int k = 2; k < 28; ++k) {
        t *= x;
        float a = t / (float(k) * float(k));
        s += a;
        if (a < 1e-8 * abs(s)) break;
    }
    return s;
}
float dilog(float x) {
    x = clamp(x, 0.0, 0.999999);
    if (x <= 0.5) return dilogSeries(x);
    return PI2_6 - log(x) * log(1.0 - x) - dilogSeries(1.0 - x);
}
float invDilog(float target) {
    target = clamp(target, 0.0, PI2_6 - 1e-6);
    if (target <= 0.0) return 0.0;
    float lo = 0.0, hi = 1.0;
    float y = target / PI2_6;
    for (int i = 0; i < 12; ++i) {
        float f = dilog(y) - target;
        if (f > 0.0) hi = y; else lo = y;
        float d = (y > 1e-6 && y < 0.999999) ? (-log(1.0 - y) / y) : 1.0;
        float ny = y - f / d;
        if (!(ny > lo && ny < hi)) ny = 0.5 * (lo + hi);
        y = ny;
    }
    return y;
}
// C3+R keep probability (GaussianPointMath::radialKeepProbability).
float radialKeep(float o, float r2) {
    o = min(o, 1.0 - 1e-6);
    float y = o * exp(-0.5 * r2);
    float hy = (y < 1e-4) ? 1.0 + 0.5 * y : -log(1.0 - y) / y;
    float ho = (o < 1e-4) ? 1.0 + 0.5 * o : -log(1.0 - o) / o;
    return clamp(hy / ho, 0.0, 1.0);
}

float sampleRadius(float o, float uu) {
    o = clamp(o, 1e-4, 1.0);
    float D = dilog(o);
    float y = min(invDilog((1.0 - uu) * D), o);
    if (y <= 0.0) return 12.0;
    return min(12.0, sqrt(max(0.0, -2.0 * log(y / o))));
}

mat3 quatToR(float w, float x, float y, float z) {
    float il = inversesqrt(max(w*w + x*x + y*y + z*z, 1e-20));
    w *= il; x *= il; y *= il; z *= il;
    return mat3(
        1.0-2.0*(y*y+z*z), 2.0*(x*y+z*w),     2.0*(x*z-y*w),
        2.0*(x*y-z*w),     1.0-2.0*(x*x+z*z), 2.0*(y*z+x*w),
        2.0*(x*z+y*w),     2.0*(y*z-x*w),     1.0-2.0*(x*x+y*y));
}

uint orderedDepthKey(float d) {
    uint b = floatBitsToUint(d);
    uint mask = ((b & 0x80000000u) != 0u) ? 0xFFFFFFFFu : 0x80000000u;
    return b ^ mask;
}

// Linear RGB via SH (3DGS eval_sh). deg 0..3; deg 0 = SH_C0*dc + 0.5.
vec3 evalSH(uint id, vec3 dc, vec3 dir) {
    int deg = int(u.ctrl2.y);
    vec3 result = u.p1.z * dc;
    if (deg >= 1) {
        int R = max(int(u.p3.w), (deg + 1) * (deg + 1) - 1);
        uint base = id * uint(3 * R);
        float x = dir.x, y = dir.y, z = dir.z;
        #define SH(k) vec3(shRest[base + uint(k)], shRest[base + uint(R) + uint(k)], shRest[base + uint(2*R) + uint(k)])
        result += -SH_C1*y*SH(0) + SH_C1*z*SH(1) - SH_C1*x*SH(2);
        if (deg >= 2) {
            float xx=x*x, yy=y*y, zz=z*z, xy=x*y, yz=y*z, xz=x*z;
            result += 1.0925484305920792*xy*SH(3)
                    - 1.0925484305920792*yz*SH(4)
                    + 0.31539156525252005*(2.0*zz-xx-yy)*SH(5)
                    - 1.0925484305920792*xz*SH(6)
                    + 0.5462742152960396*(xx-yy)*SH(7);
            if (deg >= 3) {
                result += -0.5900435899266435*y*(3.0*xx-yy)*SH(8)
                        + 2.890611442640554*xy*z*SH(9)
                        - 0.4570457994644658*y*(4.0*zz-xx-yy)*SH(10)
                        + 0.3731763325901154*z*(2.0*zz-3.0*xx-3.0*yy)*SH(11)
                        - 0.4570457994644658*x*(4.0*zz-xx-yy)*SH(12)
                        + 1.445305721320277*z*(xx-yy)*SH(13)
                        - 0.5900435899266435*x*(xx-3.0*yy)*SH(14);
            }
        }
        #undef SH
    }
    return max(result + 0.5, vec3(0.0));
}

