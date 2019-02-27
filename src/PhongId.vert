uniform highp mat4 transformationMatrix;
uniform highp mat4 projectionMatrix;
uniform mediump mat3 normalMatrix;
uniform highp vec3 light;

layout(location = 0) in highp vec4 inVertexPosition;
layout(location = 1) in mediump vec3 inNormal;
layout(location = 2) in mediump vec2 inUv;
layout(location = 3) in mediump vec3 inVertexColor;

out mediump vec3 transformedNormal;
out highp vec3 lightDirection;
out highp vec3 cameraDirection;
out mediump vec3 vFragColor;
out mediump vec2 uvFrag;

void main() {
    highp vec4 transformedPosition4 = transformationMatrix*inVertexPosition;
    highp vec3 transformedPosition = transformedPosition4.xyz/transformedPosition4.w;

    transformedNormal = normalMatrix*inNormal;

    lightDirection = normalize(light - transformedPosition);

    cameraDirection = -transformedPosition;

    gl_Position = projectionMatrix*transformedPosition4;
    uvFrag = inUv;
    vFragColor = inVertexColor;
}
