#pragma once


#include "DrawArrayGeometry.h"

class Planet;

class PointsGeometry : public DrawArrayGeometry {
public:
    explicit PointsGeometry( Planet& planet ) :
            DrawArrayGeometry( "Points", vl::PT_POINTS ),
            m_planet( planet ) {}

    void setStereo( float ratio ) {
        m_stereo = ratio;
        markGeometryDirty();
    }

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

private:
    Planet& m_planet;
    float m_stereo = 0.0f;
};



