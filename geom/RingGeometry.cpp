#include "RingGeometry.h"
#include "../Planet.h"

void RingGeometry::createGeometry() {
    resize( m_elements );
}

void RingGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    double cosLat = cos( m_latitude * vl::dDEG_TO_RAD );
    double sinLat = sin( m_latitude * vl::dDEG_TO_RAD );

    for ( int i = 0; i < m_elements; ++i ) {
        double radians = (2 * vl::dPi * i) / m_elements;
        verts->at( i ) = vl::fvec3(
                cosLat * sin( radians ),
                sinLat,
                cosLat * cos( radians ) ) * m_offset;
    }
}

void RingGeometry::updateColors() {
    auto* cols = colorBuffer();
    for ( int i = 0; i < m_elements; ++i ) {
        cols->at( i ) = vl::fvec4( 1, 1, 1, 1 );
    }
}
