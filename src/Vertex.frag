uniform highp int objectId;
uniform highp vec3 color;

layout(location = 0) out highp vec4 outColor;
layout(location = 1) out highp int outObjectId;

flat in highp int vertexID;

void main() {
    outObjectId = objectId;
    outColor = vec4(color, 1.f);
}
