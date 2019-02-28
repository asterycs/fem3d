uniform sampler2DRect Opaque;
uniform sampler2DRect TransparencyAccumulation;
uniform sampler2DRect TransparencyRevealage;

layout(location = 0) out vec4 outColor;

void main(void)
{
    vec4 sumTransparentColor = texture(TransparencyAccumulation, gl_FragCoord.xy);
    float transmittance = texture(TransparencyRevealage, gl_FragCoord.xy).r;
    vec3 averageTransparentColor = sumTransparentColor.rgb / max(sumTransparentColor.a, 0.00001);
    
    // Black background (vec3(0.f))
    vec3 transparentFinal = averageTransparentColor * (1.0f - transmittance) + vec3(0.f) * transmittance;

    vec4 opaqueColor = texture(Opaque, gl_FragCoord.xy);
    
    outColor.rgb = transparentFinal * (1.f - opaqueColor.a) + opaqueColor.rgb;
    outColor.a = 1.f;
}