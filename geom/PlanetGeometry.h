#pragma once


#include "DrawElementsGeometry.h"
#include "../Planet.h"

class Planet;

class PlanetGeometry : public DrawElementsGeometry {
public:
    explicit PlanetGeometry( Planet& sphere ) :
            DrawElementsGeometry( "Planet", vl::PT_TRIANGLE_FAN ),
            m_planet( sphere ) {}

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

    void setColorMode( int colorMode ) {
        m_colorMode = colorMode;
        markColorsDirty();
    }

    void setHighlight( std::size_t highlight ) {
        m_highlight = highlight;
        markColorsDirty();
    }

    void enablePicking( bool enabled ) {
        m_picking = enabled;
    }

    std::size_t getSize() const override {
        return DrawElementsGeometry::getSize() - m_planet.cells.size();
    }

private:
    Planet& m_planet;
    int m_colorMode = 0;
    size_t m_highlight = -1;
    bool m_picking = true;

    static float distCol( float dist );

    void colorCell( const Planet::cell& cell, vl::fvec4 rgb, vl::fvec4*& cols );
};



