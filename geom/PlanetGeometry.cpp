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
    switch ( m_planet.phase() ) {
        case 3: colFunc = &PlanetGeometry::colFuncCellPosition; break;
        case 4:
            switch ( m_colorMode ) {
                case 2: colFunc = &PlanetGeometry::colFuncPlateTypeColor; break;
                default: colFunc = &PlanetGeometry::colFuncPlatePosition; break;
            }
            break;

        case 5:
            switch ( m_colorMode ) {
                case 2: colFunc = &PlanetGeometry::colFuncCellMotion; break;
                case 3: colFunc = &PlanetGeometry::colFuncCellDivergentForce; break;
                case 4: colFunc = &PlanetGeometry::colFuncCellConvergentForce; break;
                case 5: colFunc = &PlanetGeometry::colFuncCellForce; break;
                case 6: colFunc = &PlanetGeometry::colFuncCellElevation; break;
                default: colFunc = &PlanetGeometry::colFuncDefault; break;
            }
            break;

        default:
            switch ( m_colorMode ) {
                case 2: colFunc = &PlanetGeometry::colFuncCellTemperature; break;
                case 3: colFunc = &PlanetGeometry::colFuncCellAnnualIllumination; break;
                default: colFunc = &PlanetGeometry::colFuncCellIlluminated; break;
            }
            break;
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
    if ( m_planet.phase() > 3 && cell.point == m_planet.plates[cell.plate].cell ) {
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

vl::fvec4 PlanetGeometry::colFuncCellPosition( const Planet::cell& cell ) {
    return vl::fvec4( (m_planet.points[cell.point] + 1.0) / 2.0 * vl::fvec3( 1.0, 0.8, 0.9), 1.0f );
}

vl::fvec4 PlanetGeometry::colFuncCellIlluminated( const Planet::cell& cell ) {
    vl::fvec3 rgb = m_planet.getColor( cell.elevation, cell.moisture / 100.0f );
    rgb *= (cell.illumination * 0.95f) + 0.05f;
    return vl::fvec4( rgb, 1 );
}

vl::fvec4 PlanetGeometry::colFuncCellAnnualIllumination( const Planet::cell& cell ) {
    return vl::fvec4( cell.annualIllumination, cell.annualIllumination, cell.annualIllumination, 1 );
}

vl::fvec4 PlanetGeometry::colFuncCellTemperature( const Planet::cell& cell ) {
    float t = cell.temperature / 20;
    if ( t > 0 ) {
        return vl::fvec4( 1.0, 1.0 - t, 1.0 - t, 1 );
    } else {
        return vl::fvec4( 1.0 + t, 1.0 + t, 1.0, 1 );
    }
}


vl::fvec4 PlanetGeometry::colFuncCellDivergentForce( const Planet::cell& cell ) {
    return vl::fvec4( 1.0 - cell.divergentForce, 1.0 - cell.divergentForce, 1.0, 1 );
}

vl::fvec4 PlanetGeometry::colFuncCellConvergentForce( const Planet::cell& cell ) {
    return vl::fvec4( 1.0, 1.0 - cell.convergentForce, 1.0 - cell.convergentForce, 1 );
}

vl::fvec4 PlanetGeometry::colFuncCellForce( const Planet::cell& cell ) {
    if ( cell.convergentForce >= cell.divergentForce ) {
        return vl::fvec4( 1.0, 1.0 - cell.convergentForce, 1.0 - cell.convergentForce, 1 );
    } else {
        return vl::fvec4( 1.0 - cell.divergentForce, 1.0 - cell.divergentForce, 1.0, 1 );
    }
}

vl::fvec4 PlanetGeometry::colFuncCellMotion( const Planet::cell& cell ) {
    return vl::fvec4( (cell.dir + 1.0) / 2.0 * vl::fvec3( 1.0, 0.8, 0.9), 1.0f );
}

vl::fvec4 PlanetGeometry::colFuncCellElevation( const Planet::cell& cell ) {
    float elevation = (cell.elevation + 1.0f) / 2.0f;
    return vl::fvec4( elevation, elevation, 1.0 - elevation, 1 );
}

vl::fvec4 PlanetGeometry::colFuncDefault( const Planet::cell& cell ) {
    return vl::fvec4( m_planet.getColor( cell.elevation, cell.moisture / 100.0f ), 1 );
}

vl::fvec4 PlanetGeometry::colFuncPlatePosition( const Planet::cell& cell ) {
    return vl::fvec4( (m_planet.plates[cell.plate].origin + 1.0) / 2.0 * vl::fvec3( 1.0, 0.8, 0.9), 1.0f );
}

vl::fvec4 PlanetGeometry::colFuncPlateTypeColor( const Planet::cell& cell ) {
    if ( m_planet.plates[cell.plate].oceanic) {
        return vl::fvec4( m_planet.getColor( -0.5f, 0.0f ), 1 );
    }
    return vl::fvec4( m_planet.getColor( 0.5f, 0.0f ), 1 );
}

