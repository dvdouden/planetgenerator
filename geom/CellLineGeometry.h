#pragma once


#include "DrawArrayGeometry.h"
#include "../Planet.h"

class Planet;

class CellLineGeometry : public DrawArrayGeometry {
public:
    explicit CellLineGeometry( Planet& sphere ) :
            DrawArrayGeometry( "Celllines", vl::PT_LINES ),
            m_planet( sphere ) {}

    void setIncludeBorders( bool includeBorders ) {
        if ( includeBorders != m_includeBorders ) {
            markGeometryInvalid();
            m_includeBorders = includeBorders;
        }
    }

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

private:
    bool m_includeBorders = true;
    Planet& m_planet;

    bool shouldDrawEdge( const Planet::cell& cell, const Planet::edge& edge ) const;
};



