#include "Shaders.h"

#include <Corrade/Utility/Resource.h>
#include <Corrade/Containers/Reference.h>

#include <Magnum/GL/Version.h>
#include <Magnum/GL/Shader.h>

PhongIdShader::PhongIdShader()
{
    Utility::Resource rs("fem3d-data");

#ifndef MAGNUM_TARGET_GLES
    GL::Shader vert{GL::Version::GL330, GL::Shader::Type::Vertex},
            frag{GL::Version::GL330, GL::Shader::Type::Fragment};
#else
    GL::Shader vert{GL::Version::GLES300, GL::Shader::Type::Vertex},
frag{GL::Version::GLES300, GL::Shader::Type::Fragment};
#endif
    vert.addSource(rs.get("PhongTransparentId.vert"));
    frag.addSource(rs.get("PhongTransparentId.frag"));
    CORRADE_INTERNAL_ASSERT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());
}

VertexShader::VertexShader()
{
    Utility::Resource rs("fem3d-data");

#ifndef MAGNUM_TARGET_GLES
    GL::Shader vert{GL::Version::GL330, GL::Shader::Type::Vertex},
            frag{GL::Version::GL330, GL::Shader::Type::Fragment};
#else
    GL::Shader vert{GL::Version::GLES300, GL::Shader::Type::Vertex},
frag{GL::Version::GLES300, GL::Shader::Type::Fragment};
#endif
    vert.addSource(rs.get("Vertex.vert"));
    frag.addSource(rs.get("Vertex.frag"));
    CORRADE_INTERNAL_ASSERT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());

    _colorUniform = uniformLocation("color");
    _objectIdUniform = uniformLocation("objectId");
    _transformationMatrixUniform = uniformLocation("transformationMatrix");
    _projectionMatrixUniform = uniformLocation("projectionMatrix");
}

CompositionShader::CompositionShader()
{
    Utility::Resource rs("fem3d-data");

#ifndef MAGNUM_TARGET_GLES
    GL::Shader vert{GL::Version::GL330, GL::Shader::Type::Vertex},
            frag{GL::Version::GL330, GL::Shader::Type::Fragment};
#else
    GL::Shader vert{GL::Version::GLES300, GL::Shader::Type::Vertex},
frag{GL::Version::GLES300, GL::Shader::Type::Fragment};
#endif
    vert.addSource(rs.get("Composition.vert"));
    frag.addSource(rs.get("Composition.frag"));
    CORRADE_INTERNAL_ASSERT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT(link());

}
