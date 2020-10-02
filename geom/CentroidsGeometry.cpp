#include "CentroidsGeometry.h"
#include "../Planet.h"

void CentroidsGeometry::createGeometry() {
    resize( m_planet.centers.size() );
}

void CentroidsGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    if ( m_stereo > 0.0f ) {
        for ( int i = 0; i < m_planet.centers.size(); ++i ) {
            float df = m_stereo;
            float vf = 1.0f - df;
            vl::fvec3 v = m_planet.centers[i];
            vl::dvec2 d = Planet::toStereo( v ) * df;
            v = v * vf;
            verts->at( i ) = vl::fvec3( v.x() + d.x(), v.y(), v.z() + d.y() );
        }
    } else {
        for ( int i = 0; i < m_planet.centers.size(); ++i ) {
            verts->at( i ) = m_planet.centers[i];
        }
    }
}

void CentroidsGeometry::updateColors() {
    auto* cols = colorBuffer();
    for ( int i = 0; i < m_planet.centers.size(); ++i ) {
        cols->at( i ) = vl::fvec4( 1, 1, 1, 1 );
    }
}
