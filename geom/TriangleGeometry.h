#pragma once


#include "DrawArrayGeometry.h"

class Planet;

class TriangleGeometry : public DrawArrayGeometry {
public:
    explicit TriangleGeometry( Planet& planet ) :
            DrawArrayGeometry( "Triangles", vl::PT_TRIANGLES ),
            m_planet( planet ) {}

    void setStereo( float ratio ) {
        m_stereo = ratio;
        markGeometryDirty();
    }

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

    std::size_t getSize() const override {
        return DrawArrayGeometry::getSize() / 3;
    }

private:
    Planet& m_planet;
    float m_stereo = 0.0f;
};



