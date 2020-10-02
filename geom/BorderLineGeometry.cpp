#include "BorderLineGeometry.h"
#include "../Planet.h"

void BorderLineGeometry::createGeometry() {
    // calculate number of borders
    int borderCount = 0;
    m_cellIdx.clear();
    for ( const auto& cell : m_sphere.cells ) {
        for ( const auto& edge : cell.edges ) {
            // prevent borders from being rendered twice
            if ( m_sphere.cells[edge.neighbor].plate > cell.plate ) {
                ++borderCount;
                // store the cell id
                m_cellIdx.push_back( cell.point );
            }
        }
    }
    resize( borderCount * 2 );
}

void BorderLineGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    int borderIdx = 0;
    for ( const auto& cell : m_sphere.cells ) {
        for ( const auto& edge : cell.edges ) {
            if ( m_sphere.cells[edge.neighbor].plate > cell.plate ) {
                verts->at(borderIdx++) = m_sphere.centers[edge.a];
                verts->at(borderIdx++) = m_sphere.centers[edge.b];
            }
        }
    }
    // elevate all points slightly to prevent Z fighting
    for ( int i = 0; i < verts->size(); ++i ) {
        verts->at(i) *= 1.01f;
    }
}

void BorderLineGeometry::updateColors() {
    auto* cols = colorBuffer();
    int borderIdx = 0;
    for (std::size_t cell : m_cellIdx) {
        float illumination = (1.0f - m_sphere.cells[cell].illumination) * 0.95f + 0.05f;
        cols->at(borderIdx++) = vl::fvec4( illumination, illumination, illumination, 1 );
        cols->at(borderIdx++) = vl::fvec4( illumination, illumination, illumination, 1 );
    }
}
