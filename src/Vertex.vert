uniform highp mat4 transformationMatrix;
uniform highp mat4 projectionMatrix;

layout(location = 0) in highp vec4 position;

void main() {
    highp vec4 transformedPosition4 = transformationMatrix*position;
    highp vec3 transformedPosition = transformedPosition4.xyz/transformedPosition4.w;

    gl_Position = projectionMatrix*transformedPosition4;
}
