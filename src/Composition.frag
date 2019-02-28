uniform sampler2D Opaque;
uniform sampler2D TransparencyAccumulation;
uniform sampler2D TransparencyRevealage;
uniform highp ivec2 viewportSize;

layout(location = 0) out highp vec4 outColor;

void main(void)
{
    highp vec2 texCoord = gl_FragCoord.xy/vec2(viewportSize);
    highp vec4 sumTransparentColor = texture(TransparencyAccumulation, texCoord);
    highp float transmittance = texture(TransparencyRevealage,  texCoord).r;
    highp vec3 averageTransparentColor = sumTransparentColor.rgb / max(sumTransparentColor.a, 0.00001);
    
    // Black background (vec3(0.f))
    highp vec3 transparentFinal = averageTransparentColor * (1.0f - transmittance) + vec3(0.f) * transmittance;

    highp vec4 opaqueColor = texture(Opaque, texCoord);
    
    outColor.rgb = transparentFinal * (1.f - opaqueColor.a) + opaqueColor.rgb;
    outColor.a = 1.f;
}