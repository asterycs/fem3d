uniform lowp vec3 ambientColor;

in mediump vec3 transformedNormal;
in highp vec3 lightDirection;
in highp vec3 cameraDirection;
in mediump vec3 vFragColor;
in mediump vec2 uvFrag;

layout(location = 0) out highp vec4 outColor;
layout(location = 1) out highp int outObjectId;
layout(location = 2) out highp vec4 outSumColor;
layout(location = 3) out highp vec4 outSumWeight;

void main() {
    mediump vec3 normalizedTransformedNormal = normalize(transformedNormal);
    highp vec3 normalizedLightDirection = normalize(lightDirection);

    mediump vec4 color;
    color.rgb = ambientColor;

    lowp float intensity = max(0.0, abs(dot(normalizedTransformedNormal, normalizedLightDirection)));
    color.rgb += vFragColor*intensity;

    if(intensity > 0.001) {
        highp vec3 reflection = reflect(-normalizedLightDirection, normalizedTransformedNormal);
        mediump float specularity = pow(max(0.0, abs(dot(normalize(cameraDirection), reflection))), 80.0);
        color.rgb += vec3(1.0)*specularity;
    }

    color.a = 0.2;
    outColor = color;
    outObjectId = -1;
    //float closestEdge = min(uvFrag.x, min(uvFrag.y, 1.f - uvFrag.x - uvFrag.y));

    float viewDepth = abs(1.0 / gl_FragCoord.w);

    float depthScale = 0.5f;
    float linearDepth = viewDepth * depthScale;
    float weight = clamp(0.03 / (1e-5 + pow(linearDepth, 4.0)), 1e-2, 3e3);

    // Blend Func: GL_ONE, GL_ONE
    outSumColor = vec4(color.rgb * color.a, color.a) * weight;

    // Blend Func: GL_ZERO, GL_ONE_MINUS_SRC_ALPHA
    outSumWeight = vec4(color.a);
}
