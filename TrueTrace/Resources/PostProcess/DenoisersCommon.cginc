#include "../MainCompute/CommonStructs.cginc"
int screen_width;
int screen_height;
int UpscalerMethod;
bool DiffRes;
float4x4 CamToWorld;
float4x4 CamInvProj;
float4x4 CamToWorldPrev;
float4x4 CamInvProjPrev;

RWTexture2D<float4> _DebugTex;

Texture2D<float4> ScreenSpaceInfoPrev;
Texture2D<float4> ScreenSpaceInfo;
SamplerState my_linear_clamp_sampler;

uint GetBounceData(uint A) {
    return (A & 0x7C000000) >> 26;
}

StructuredBuffer<ColData> GlobalColorsRead;

struct Ray {
    float3 origin;
    float3 direction;
};

RWStructuredBuffer<RayData> GlobalRaysMini;
Ray CreateRay(float3 origin, float3 direction) {
    Ray ray;
    ray.origin = origin;
    ray.direction = direction;
    return ray;
}

inline Ray CreateCameraRay(float2 uv) {
    // Transform the camera origin to world space
    float3 origin = mul(CamToWorld, float4(0.0f, 0.0f, 0.0f, 1.0f)).xyz;

    // Invert the perspective projection of the view-space position
    float3 direction = mul(CamInvProj, float4(uv, 0.0f, 1.0f)).xyz;
    // Transform the direction from camera to world space and normalize
    direction = mul(CamToWorld, float4(direction, 0.0f)).xyz;
    direction = normalize(direction);

    return CreateRay(origin, direction);
}

inline Ray CreateCameraRayPrev(float2 uv) {
    // Transform the camera origin to world space
    float3 origin = mul(CamToWorldPrev, float4(0.0f, 0.0f, 0.0f, 1.0f)).xyz;

    // Invert the perspective projection of the view-space position
    float3 direction = mul(CamInvProjPrev, float4(uv, 0.0f, 1.0f)).xyz;
    // Transform the direction from camera to world space and normalize
    direction = mul(CamToWorldPrev, float4(direction, 0.0f)).xyz;
    direction = normalize(direction);

    return CreateRay(origin, direction);
}


inline float luminance(in float3 color) {
    return dot(color, float3(0.299, 0.587, 0.114));
}

float3 FromColorSpecPacked(uint A) {
    return float3(
        (A & 0x3FF) / 1022.0f,
        ((A >> 10) & 0x3FF) / 1022.0f,
        ((A >> 20) & 0x3)
        );
}

inline uint packRGBE(const float3 v) {
    float3 va = max(0, v);
    float max_abs = max(va.r, max(va.g, va.b));
    [branch]if (max_abs == 0) return 0;
    int exponent = (int)(log2(max_abs));
    uint e = clamp(exponent + 20, 0, 31);
    float scale = exp2(-exponent) * 256.0;
    uint3 vu = (uint3)min(511.0f.xxx, round(va * scale));
    return (e << 27) | (vu.r) | (vu.g << 9) | (vu.b << 18);
}

inline float3 unpackRGBE(const uint x) {
    float scale = exp2((int)(x >> 27) - 20) * (1.0f / 256.0f);
    uint r =  x        & 0x1FF;
    uint g = (x >> 9 ) & 0x1FF;
    uint b = (x >> 18) & 0x1FF;
    return float3(r, g, b) * scale;
}
inline uint octahedral_32(float3 nor) {
    float oct = rcp(abs(nor.x) + abs(nor.y) + abs(nor.z));
    float t = saturate(-nor.z);
    nor.xy = (nor.xy + (nor.xy > 0.0f ? t : -t)) * oct;
    uint2 d = uint2(round(32767.5 + nor.xy*32767.5));  
    return d.x|(d.y<<16u);
}

inline float3 i_octahedral_32(uint data ) {
    uint2 iv = uint2( data, data>>16u ) & 65535u; 
    float2 v = iv * (1.0f / 32767.5f) - 1.0f;
    float3 nor = float3(v, 1.0f - abs(v.x) - abs(v.y)); // Rune Stubbe's version,
    float t = max(-nor.z,0.0);                     // much faster than original
    nor.xy += (nor.xy>=0.0)?-t:t;                     // implementation of this
    return nor * rsqrt(dot(nor, nor));
}