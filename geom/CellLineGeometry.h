#pragma once


#include "DrawArrayGeometry.h"

class Planet;

class CellLineGeometry : public DrawArrayGeometry {
public:
    explicit CellLineGeometry( Planet& sphere ) :
            DrawArrayGeometry( "Celllines", vl::PT_LINES ),
            m_planet( sphere ) {}

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

private:
    Planet& m_planet;
};



