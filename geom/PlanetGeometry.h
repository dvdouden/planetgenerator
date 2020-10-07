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
        m_highlightDirty = false;
        markColorsDirty();
    }

    void setHighlight( std::size_t highlight ) {
        if ( highlight == m_highlight ) {
            return;
        }
        m_oldHighlight = m_highlight;
        m_highlight = highlight;
        markColorsDirty();
        m_highlightDirty = true;
    }

    void enablePicking( bool enabled ) {
        m_picking = enabled;
    }

    std::size_t getSize() const override {
        return DrawElementsGeometry::getSize() - m_planet.cells.size();
    }

private:

    typedef vl::fvec4 (PlanetGeometry::*colFunc)( const Planet::cell& );

    vl::fvec4 colFunc1( const Planet::cell& cell );
    vl::fvec4 colFunc2( const Planet::cell& cell );
    vl::fvec4 colFunc3( const Planet::cell& cell );
    vl::fvec4 colFunc4( const Planet::cell& cell );
    vl::fvec4 colFunc5( const Planet::cell& cell );
    vl::fvec4 colFunc6( const Planet::cell& cell );
    vl::fvec4 colFunc7( const Planet::cell& cell );
    vl::fvec4 colFunc8( const Planet::cell& cell );
    vl::fvec4 colFuncDefault( const Planet::cell& cell );


    Planet& m_planet;
    int m_colorMode = 0;
    size_t m_highlight = -1;
    size_t m_oldHighlight = -1;
    bool m_highlightDirty = false;
    bool m_picking = true;
    std::vector<size_t> m_offsets;

    static float distCol( float dist );

    void colorCell( const Planet::cell& cell, vl::fvec4 rgb, vl::fvec4*& cols );


    vl::fvec4 colFunc9( const Planet::cell& cell );

    vl::fvec4 colFuncPlateColor( const Planet::cell& cell );
};



