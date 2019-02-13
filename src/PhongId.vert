uniform highp mat4 transformationMatrix;
uniform highp mat4 projectionMatrix;
uniform mediump mat3 normalMatrix;
uniform highp vec3 light;

layout(location = 0) in highp vec4 position;
layout(location = 1) in mediump vec3 normal;
layout(location = 2) in mediump vec2 uv;
layout(location = 3) in mediump vec3 vertexColor;

out mediump vec3 transformedNormal;
out highp vec3 lightDirection;
out highp vec3 cameraDirection;
out mediump vec3 color;
out mediump vec2 uvFrag;

void main() {
    highp vec4 transformedPosition4 = transformationMatrix*position;
    highp vec3 transformedPosition = transformedPosition4.xyz/transformedPosition4.w;

    transformedNormal = normalMatrix*normal;

    lightDirection = normalize(light - transformedPosition);

    cameraDirection = -transformedPosition;

    gl_Position = projectionMatrix*transformedPosition4;
    uvFrag = uv;
    color = vertexColor;
}
