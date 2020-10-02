#pragma once


#include "DrawArrayGeometry.h"

class Planet;

class CentroidsGeometry : public DrawArrayGeometry {
public:
    explicit CentroidsGeometry( Planet& planet ) :
            DrawArrayGeometry( "Centroids", vl::PT_POINTS ),
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



