uniform highp int highlightedVertexId;
uniform highp int objectId;

uniform mediump vec3 color;

layout(location = 0) out lowp vec4 fragmentColor;
layout(location = 1) out lowp int fragmentObjectId;

flat in highp int vertexID;

void main() {
    fragmentColor = vec4(color, 1.f);
    fragmentObjectId = objectId;
}
