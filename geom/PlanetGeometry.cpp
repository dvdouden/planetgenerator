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

    resize( vtxCount, idxCount );
    setBoundingSphere( vl::Sphere( vl::vec3(), 1.0 ) );
    setBoundingBox( vl::AABB(vl::vec3(), 1.0));
}

void PlanetGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    auto* idxBuf = indexBuffer();
    int idx = 0;
    int vIdx = 0;
    auto lastCellPoint = m_planet.cells.back().point;
    for ( const auto & cell : m_planet.cells ) {
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
    auto* v = cols->begin();
    if ( m_colorMode == 1 ) {
        for ( const auto & cell : m_planet.cells ) {
            vl::fvec4 rgb = vl::fvec4( (m_planet.points[cell.point] + 1.0) / 2.0 * vl::fvec3( 1.0, 0.8, 0.9), 1.0f );
            colorCell( cell, rgb, v );
        }
    } else if ( m_colorMode == 2 ) {
        for ( const auto & cell : m_planet.cells ) {
            vl::fvec4 rgb = vl::fvec4( distCol( cell.dMnt ), 0, 0, 1 );
            colorCell( cell, rgb, v );
        }
    } else if ( m_colorMode == 3 ) {
        for ( const auto & cell : m_planet.cells ) {
            vl::fvec4 rgb = vl::fvec4( 0, distCol( cell.dCst ), 0, 1 );
            colorCell( cell, rgb, v );
        }
    } else if ( m_colorMode == 4 ) {
        for ( const auto & cell : m_planet.cells ) {
            vl::fvec4 rgb = vl::fvec4( 0, 0, distCol( cell.dOcn ), 1 );
            colorCell( cell, rgb, v );
        }
    } else if ( m_colorMode == 5 ) {
        for ( const auto & cell : m_planet.cells ) {
            vl::fvec4 rgb = vl::fvec4( distCol( cell.dMnt ), distCol( cell.dCst ), distCol( cell.dOcn ), 1 );
            colorCell( cell, rgb, v );
        }
    } else if ( m_colorMode == 6 ) {
        for ( const auto & cell : m_planet.cells ) {
            vl::fvec3 rgb = m_planet.getColor( cell.elevation, cell.moisture / 100.0f );
            rgb *= (cell.illumination * 0.95f) + 0.05f;
            vl::fvec4 rgbw( rgb, 1 );
            colorCell( cell, rgbw, v );
        }
    } else if ( m_colorMode == 7 ) {
        for ( const auto & cell : m_planet.cells ) {
            vl::fvec4 rgb = vl::fvec4( cell.annualIllumination, cell.annualIllumination, cell.annualIllumination, 1 );
            colorCell( cell, rgb, v );
        }
    } else if ( m_colorMode == 8 ) {
        for ( const auto & cell : m_planet.cells ) {
            float t = cell.temperature / 20;
            vl::fvec4 rgb;
            if ( t > 0 ) {
                rgb = vl::fvec4( 1.0, 1.0 - t, 1.0 - t, 1 );
            } else {
                rgb = vl::fvec4( 1.0 + t, 1.0 + t, 1.0, 1 );
            }
            colorCell( cell, rgb, v );
        }
    } else {
        for ( const auto & cell : m_planet.cells ) {
            vl::fvec4 rgb = vl::fvec4( m_planet.getColor( cell.elevation, cell.moisture / 100.0f ), 1 );
            colorCell( cell, rgb, v );
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
        rgb *= 1.1;
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
