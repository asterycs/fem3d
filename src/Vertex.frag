uniform highp int objectId;
uniform highp vec3 color;

layout(location = 0) out highp vec4 outColor;
layout(location = 1) out highp int outObjectId;

// Seems like a WebGL bug. Without these declarations
// the targets remain uncleared
layout(location = 2) out highp vec4 outSumColor;
layout(location = 3) out highp vec4 outSumWeight;

flat in highp int vertexID;

void main() {
    outObjectId = objectId;
    outColor = vec4(color, 1.f);
}
