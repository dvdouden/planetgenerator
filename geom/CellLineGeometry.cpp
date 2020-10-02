#include "CellLineGeometry.h"
#include "../Planet.h"

void CellLineGeometry::createGeometry() {
    // calculate number of non-borders
    int edgeCount = 0;
    for ( const auto& cell : m_planet.cells ) {
        for ( const auto& edge : cell.edges ) {
            if ( !edge.plateBorder ) {
                ++edgeCount;
            }
        }
    }
    resize( edgeCount * 2 );
}

void CellLineGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    int edgeIdx = 0;
    for ( const auto& cell : m_planet.cells ) {
        for ( const auto& edge : cell.edges ) {
            if ( !edge.plateBorder ) {
                verts->at(edgeIdx++) = m_planet.centers[edge.a];
                verts->at(edgeIdx++) = m_planet.centers[edge.b];
            }
        }
    }
    // elevate all points slightly to prevent Z fighting
    for ( int i = 0; i < verts->size(); ++i ) {
        verts->at(i) *= 1.01f;
    }
}

void CellLineGeometry::updateColors() {
    auto* cols = colorBuffer();
    int edgeIdx = 0;
    for ( const auto& cell : m_planet.cells ) {
        float illumination = cell.illumination * 0.95f + 0.05f;
        for ( const auto& edge : cell.edges ) {
            if ( !edge.plateBorder ) {
                cols->at( edgeIdx++ ) = vl::fvec4( illumination, illumination, illumination, 1 );
                cols->at( edgeIdx++ ) = vl::fvec4( illumination, illumination, illumination, 1 );
            }
        }
    }
}
