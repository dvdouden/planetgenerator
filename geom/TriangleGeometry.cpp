#include "TriangleGeometry.h"
#include "../Planet.h"

void TriangleGeometry::createGeometry() {
    resize( m_planet.triangles.size() );
}

void TriangleGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    if ( m_stereo > 0.0f ) {
        for ( int i = 0; i < m_planet.triangles.size(); ++i ) {
            float df = m_stereo;
            float vf = 1.0f - df;
            vl::fvec3 v = m_planet.points[m_planet.triangles[i]];
            vl::dvec2 d = Planet::toStereo( v ) * df;
            v = v * vf;
            verts->at( i ) = vl::fvec3( v.x() + d.x(), v.y(), v.z() + d.y() );
        }
    } else {
        for ( int i = 0; i < m_planet.triangles.size(); ++i ) {
            verts->at( i ) = m_planet.points[m_planet.triangles[i]];
        }
    }
}

void TriangleGeometry::updateColors() {
    auto* cols = colorBuffer();
    for ( int i = 0; i < m_planet.triangles.size(); ++i ) {
        cols->at( i ) = vl::fvec4( (m_planet.points[m_planet.triangles[i]] + 1.0) / 2.0 * vl::fvec3( 1.0, 0.8, 0.9), 1.0f );
    }
}
