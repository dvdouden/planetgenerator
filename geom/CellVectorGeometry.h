#pragma once


#include "DrawArrayGeometry.h"

class Planet;

class CellVectorGeometry : public DrawArrayGeometry {
public:
    explicit CellVectorGeometry( Planet& sphere ) :
            DrawArrayGeometry( "Cellvectors", vl::PT_LINES ),
            m_sphere( sphere ) {}

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

private:
    Planet& m_sphere;
};



