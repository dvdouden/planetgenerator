#include "PlanetGeometry.h"
#include "../Planet.h"

void PlanetGeometry::createGeometry() {
    // calc total tris required
    std::size_t vtxCount = 0;
    std::size_t idxCount = 0;
    for ( const auto& cell : m_planet.cells ) {
        idxCount += cell.edges.size() + 3; // 1 for the center, one to close the fan, one for the primitive restart command
        vtxCount += cell.edges.size() + 1; // 1 for the center
    }
    idxCount--; // we don't need a primitive restart command at the end
    m_offsets.resize( m_planet.cells.size() );

    resize( vtxCount, idxCount );
    setBoundingSphere( vl::Sphere( vl::vec3(), 1.0 ) );
    setBoundingBox( vl::AABB(vl::vec3(), 1.0));
    m_oldHighlight = -1;
    m_highlight = -1;
    m_highlightDirty = false;
}

void PlanetGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    auto* idxBuf = indexBuffer();
    int idx = 0;
    int vIdx = 0;
    auto lastCellPoint = m_planet.cells.back().point;
    for ( const auto & cell : m_planet.cells ) {
        m_offsets[ cell.point ] = vIdx; // store vertex index in offsets lookup table
        verts->at( vIdx ) = m_planet.points[ cell.point ];
        idxBuf->at( idx++ ) = vIdx;
        ++vIdx;
        for ( const auto & edge : cell.edges ) {
            verts->at( vIdx ) = m_planet.centers[ edge.a ];
            idxBuf->at( idx++ ) = vIdx;
            ++vIdx;
        }
        idxBuf->at( idx++ ) = vIdx - cell.edges.size();

        if ( cell.point != lastCellPoint ) {
            idxBuf->at( idx++ ) = primitiveRestartIndex();
        }
    }/*
    // verify index buffer
    for ( std::size_t i = 0; i < idxBuf->size(); ++i ) {
        if ( idxBuf->at( i ) < 0 || idxBuf->at( i ) >= verts->size() ) {
            printf( "OH NO: [%d] 0 > %d > %d\n", i, idxBuf->at(i), verts->size() );
        }
    }*/
}

void PlanetGeometry::updateColors() {
    auto* cols = colorBuffer();


    colFunc colFunc;
    switch ( m_colorMode ) {
        case 1: colFunc = &PlanetGeometry::colFunc1; break;
        case 2: colFunc = &PlanetGeometry::colFunc2; break;
        case 3: colFunc = &PlanetGeometry::colFunc3; break;
        case 4: colFunc = &PlanetGeometry::colFunc4; break;
        case 5: colFunc = &PlanetGeometry::colFunc5; break;
        case 6: colFunc = &PlanetGeometry::colFunc6; break;
        case 7: colFunc = &PlanetGeometry::colFunc7; break;
        case 8: colFunc = &PlanetGeometry::colFunc8; break;
        default: colFunc = &PlanetGeometry::colFuncDefault; break;
    }

    if ( m_highlightDirty ) {
        if ( m_oldHighlight != -1 ) {
            auto* v = cols->begin() + m_offsets[m_oldHighlight];
            colorCell( m_planet.cells[m_oldHighlight], (this->*colFunc)( m_planet.cells[m_oldHighlight] ), v );
        }
        if ( m_highlight != -1 ) {
            auto* v = cols->begin() + m_offsets[m_highlight];
            colorCell( m_planet.cells[m_highlight], (this->*colFunc)( m_planet.cells[m_highlight] ), v );
        }
        m_highlightDirty = false;
    } else {
        auto* v = cols->begin();
        for ( const auto& cell : m_planet.cells ) {
            colorCell( cell, (this->*colFunc)( cell ), v );
        }
    }
}

float PlanetGeometry::distCol( float dist ) {
    if ( dist <= 1e-3 ) {
        return 1.0f;
    }
    if ( dist >= 0.5f ) {
        return 0.0f;
    }
    return 0.75f - dist;
}

void PlanetGeometry::colorCell( const Planet::cell& cell, vl::fvec4 rgb, vl::fvec4*& v ) {
    if ( cell.point == m_planet.plates[cell.plate].cell ) {
        rgb *= 1.25;
    }
    if ( m_picking && m_highlight != cell.point ) {
        rgb /= 2;
        rgb.w() = 1;
    }
    // center first
    *v = rgb;
    ++v;
    // then edges
    for ( const auto & edge : cell.edges ) {
        *v = rgb;
        ++v;
    }
}

vl::fvec4 PlanetGeometry::colFunc1( const Planet::cell& cell ) {
    return vl::fvec4( (m_planet.points[cell.point] + 1.0) / 2.0 * vl::fvec3( 1.0, 0.8, 0.9), 1.0f );
}

vl::fvec4 PlanetGeometry::colFunc2( const Planet::cell& cell ) {
    return vl::fvec4( distCol( cell.dMnt ), 0, 0, 1 );
}

vl::fvec4 PlanetGeometry::colFunc3( const Planet::cell& cell ) {
    return vl::fvec4( 0, distCol( cell.dCst ), 0, 1 );
}

vl::fvec4 PlanetGeometry::colFunc4( const Planet::cell& cell ) {
    return vl::fvec4( 0, 0, distCol( cell.dOcn ), 1 );
}

vl::fvec4 PlanetGeometry::colFunc5( const Planet::cell& cell ) {
    return vl::fvec4( distCol( cell.dMnt ), distCol( cell.dCst ), distCol( cell.dOcn ), 1 );
}

vl::fvec4 PlanetGeometry::colFunc6( const Planet::cell& cell ) {
    vl::fvec3 rgb = m_planet.getColor( cell.elevation, cell.moisture / 100.0f );
    rgb *= (cell.illumination * 0.95f) + 0.05f;
    return vl::fvec4( rgb, 1 );
}

vl::fvec4 PlanetGeometry::colFunc7( const Planet::cell& cell ) {
    return vl::fvec4( cell.annualIllumination, cell.annualIllumination, cell.annualIllumination, 1 );
}

vl::fvec4 PlanetGeometry::colFunc8( const Planet::cell& cell ) {
    float t = cell.temperature / 20;
    if ( t > 0 ) {
        return vl::fvec4( 1.0, 1.0 - t, 1.0 - t, 1 );
    } else {
        return vl::fvec4( 1.0 + t, 1.0 + t, 1.0, 1 );
    }
}

vl::fvec4 PlanetGeometry::colFuncDefault( const Planet::cell& cell ) {
    return vl::fvec4( m_planet.getColor( cell.elevation, cell.moisture / 100.0f ), 1 );
}
