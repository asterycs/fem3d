uniform lowp vec3 ambientColor;

in mediump vec3 transformedNormal;
in highp vec3 lightDirection;
in highp vec3 cameraDirection;
in mediump vec3 color;
in mediump vec2 uvFrag;

layout(location = 0) out lowp vec4 fragmentColor;
layout(location = 1) out highp int objectId;

void main() {
    mediump vec3 normalizedTransformedNormal = normalize(transformedNormal);
    highp vec3 normalizedLightDirection = normalize(lightDirection);

    fragmentColor.rgb = ambientColor;

    lowp float intensity = max(0.0, abs(dot(normalizedTransformedNormal, normalizedLightDirection)));
    fragmentColor.rgb += color*intensity;

    if(intensity > 0.001) {
        highp vec3 reflection = reflect(-normalizedLightDirection, normalizedTransformedNormal);
        mediump float specularity = pow(max(0.0, abs(dot(normalize(cameraDirection), reflection))), 80.0);
        fragmentColor.rgb += vec3(1.0)*specularity;
    }

    fragmentColor.a = 0.2;
    objectId = -1;
    //float closestEdge = min(uvFrag.x, min(uvFrag.y, 1.f - uvFrag.x - uvFrag.y));
}
