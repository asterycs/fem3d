uniform highp mat4 transformationMatrix;
uniform highp mat4 projectionMatrix;
uniform highp mat3 normalMatrix;
uniform highp vec3 light;

layout(location = 0) in highp vec4 inVertexPosition;
layout(location = 1) in highp vec3 inNormal;
layout(location = 2) in highp vec3 inVertexColor;

out highp vec3 transformedNormal;
out highp vec3 lightDirection;
out highp vec3 cameraDirection;
out highp vec3 vFragColor;

void main() {
    highp vec4 transformedPosition4 = transformationMatrix*inVertexPosition;
    highp vec3 transformedPosition = transformedPosition4.xyz/transformedPosition4.w;

    transformedNormal = normalMatrix*inNormal;

    lightDirection = normalize(light - transformedPosition);

    cameraDirection = -transformedPosition;

    gl_Position = projectionMatrix*transformedPosition4;
    vFragColor = inVertexColor;
}
