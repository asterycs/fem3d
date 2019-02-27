#ifndef FEM3D_SHADERS_H
#define FEM3D_SHADERS_H

#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>

#include <Magnum/GL/AbstractShaderProgram.h>

#include <Magnum/GL/RectangleTexture.h>

using namespace Magnum;

class PhongIdShader: public GL::AbstractShaderProgram {
public:
    typedef GL::Attribute<0, Vector3> Position;
    typedef GL::Attribute<1, Vector3> Normal;
    typedef GL::Attribute<2, Vector2> UV;
    typedef GL::Attribute<3, Vector3> VertexColor;

    enum: UnsignedInt {
        ColorOutput = 0,
        ObjectIdOutput = 1,
        ColorBlendOutput = 2,
        ColorWeightOutput = 3
    };

    explicit PhongIdShader();

    PhongIdShader& setLightPosition(const Vector3& position) {
        setUniform(_lightPositionUniform, position);
        return *this;
    }

    PhongIdShader& setAmbientColor(const Color3& color) {
        setUniform(_ambientColorUniform, color);
        return *this;
    }

    PhongIdShader& setTransformationMatrix(const Matrix4& matrix) {
        setUniform(_transformationMatrixUniform, matrix);
        return *this;
    }

    PhongIdShader& setNormalMatrix(const Matrix3x3& matrix) {
        setUniform(_normalMatrixUniform, matrix);
        return *this;
    }

    PhongIdShader& setProjectionMatrix(const Matrix4& matrix) {
        setUniform(_projectionMatrixUniform, matrix);
        return *this;
    }

private:
    Int     _lightPositionUniform,
            _ambientColorUniform,
            _transformationMatrixUniform,
            _normalMatrixUniform,
            _projectionMatrixUniform;
};

class VertexShader: public GL::AbstractShaderProgram {
public:
    typedef GL::Attribute<0, Vector3> Position;

    enum: UnsignedInt {
        ColorOutput = 0,
        ObjectIdOutput = 1
    };

    explicit VertexShader();

    VertexShader& setColor(const Vector3& color) {
        setUniform(_colorUniform, color);
        return *this;
    }

    VertexShader& setObjectId(Int id) {
        setUniform(_objectIdUniform, id);
        return *this;
    }

    VertexShader& setTransformationMatrix(const Matrix4& matrix) {
        setUniform(_transformationMatrixUniform, matrix);
        return *this;
    }

    VertexShader& setProjectionMatrix(const Matrix4& matrix) {
        setUniform(_projectionMatrixUniform, matrix);
        return *this;
    }

private:
    Int _objectIdUniform,
            _transformationMatrixUniform,
            _projectionMatrixUniform,
            _colorUniform;
};

class CompositionShader: public GL::AbstractShaderProgram {
public:
    typedef GL::Attribute<0, Vector3> Position;

    enum: UnsignedInt {
        ColorOutput = 0
    };

    enum: UnsignedInt {
        Tex0 = 0,
        Tex1 = 1
    };

    explicit CompositionShader();

    CompositionShader& setTex0(GL::RectangleTexture& texture) {
        setUniform(uniformLocation("ColorTex0"), Tex0);
        texture.bind(Tex0);
        return *this;
    }

    CompositionShader& setTex1(GL::RectangleTexture& texture) {
        setUniform(uniformLocation("ColorTex1"), Tex1);
        texture.bind(Tex1);
        return *this;
    }

private:
};


#endif //FEM3D_SHADERS_H
