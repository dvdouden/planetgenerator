#pragma once

#include <string>

#include <vlGraphics/Geometry.hpp>

class VLGRAPHICS_EXPORT DrawArrayGeometry : public vl::Geometry {
    VL_INSTRUMENT_CLASS(DrawArrayGeometry, vl::Geometry )

public:
    explicit DrawArrayGeometry( const std::string& name, vl::EPrimitiveType primitiveType );

    void setEnabled( bool enabled );
    bool isEnabled() const;

    virtual void createGeometry() = 0;
    virtual void updateGeometry() = 0;
    virtual void updateColors() = 0;

    void markGeometryInvalid() {
        m_geometryInvalid = true;
    }

    void markGeometryDirty() {
        m_geometryDirty = true;
    }

    void markColorsDirty() {
        m_colorsDirty = true;
    }

    void update();

    virtual std::size_t getSize() const {
        return vertexArray()->size();
    }

protected:

    void resize( std::size_t size );

    vl::ArrayFloat4* colorBuffer() {
        return colorArray()->as<vl::ArrayFloat4>();
    }
    vl::ArrayFloat3* vertexBuffer() {
        return vertexArray()->as<vl::ArrayFloat3>();
    }

private:
    bool m_geometryInvalid = true;
    bool m_colorsDirty = true;
    bool m_geometryDirty = true;
};



