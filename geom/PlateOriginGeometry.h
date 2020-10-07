#pragma once


#include "DrawArrayGeometry.h"
#include "../Planet.h"

class Planet;

class PlateOriginGeometry : public DrawArrayGeometry {
public:
    explicit PlateOriginGeometry( Planet& planet ) :
            DrawArrayGeometry( "PlateOrigins", vl::PT_LINES ),
            m_planet( planet ) {}

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

private:
    Planet& m_planet;
};



