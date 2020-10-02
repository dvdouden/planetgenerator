#include "CellVectorGeometry.h"
#include "../Planet.h"

void CellVectorGeometry::createGeometry() {
    resize( m_sphere.cells.size() * 2 );
}

void CellVectorGeometry::updateGeometry() {
    auto* verts = vertexBuffer();

    int vectorIdx = 0;
    for ( const auto& cell : m_sphere.cells ) {
        const vl::fvec3& v = m_sphere.points[cell.point];
        verts->at(vectorIdx++) = v;
        verts->at(vectorIdx++) = v + (cell.dir * m_sphere.nScale);
    }
    // elevate all points slightly to prevent Z fighting
    for ( int i = 0; i < verts->size(); ++i ) {
        verts->at(i) *= 1.01f;
    }
}

void CellVectorGeometry::updateColors() {
    auto* cols = colorBuffer();
    int vectorIdx = 0;
    for ( const auto& cell : m_sphere.cells ) {
        cols->at( vectorIdx++ ) = vl::fvec4( cell.color * 1.5, 1 );
        cols->at( vectorIdx++ ) = vl::fvec4( 1, 1, 1, 1 );
    }
}
