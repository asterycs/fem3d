uniform highp int objectId;
uniform highp vec3 color;

layout(location = 1) out highp int outObjectId;
layout(location = 2) out highp vec4 outSumColor;
layout(location = 3) out highp vec4 outSumWeight;

flat in highp int vertexID;

void main() {
    outObjectId = objectId;

    outSumColor = vec4(color, 1.f);;
    outSumWeight = vec4(0.f);
}
