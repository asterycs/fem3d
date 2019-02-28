uniform sampler2D Opaque;
uniform sampler2D TransparencyAccumulation;
uniform sampler2D TransparencyRevealage;
uniform ivec2 viewportSize;

layout(location = 0) out vec4 outColor;

void main(void)
{
    vec2 texCoord = gl_FragCoord.xy/viewportSize;
    vec4 sumTransparentColor = texture(TransparencyAccumulation, texCoord);
    float transmittance = texture(TransparencyRevealage,  texCoord).r;
    vec3 averageTransparentColor = sumTransparentColor.rgb / max(sumTransparentColor.a, 0.00001);
    
    // Black background (vec3(0.f))
    vec3 transparentFinal = averageTransparentColor * (1.0f - transmittance) + vec3(0.f) * transmittance;

    vec4 opaqueColor = texture(Opaque, texCoord);
    
    outColor.rgb = transparentFinal * (1.f - opaqueColor.a) + opaqueColor.rgb;
    outColor.a = 1.f;
}