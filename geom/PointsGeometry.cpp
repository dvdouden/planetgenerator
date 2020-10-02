#include "PointsGeometry.h"
#include "../Planet.h"

void PointsGeometry::createGeometry() {
    resize( m_planet.points.size() );
}

void PointsGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    if ( m_stereo > 0.0f ) {
        for ( int i = 0; i < m_planet.points.size(); ++i ) {
            float df = m_stereo;
            float vf = 1.0f - df;
            vl::fvec3 v = m_planet.points[i];
            vl::dvec2 d = Planet::toStereo( v ) * df;
            v = v * vf;
            verts->at( i ) = vl::fvec3( v.x() + d.x(), v.y(), v.z() + d.y() );
        }
    } else {
        for ( int i = 0; i < m_planet.points.size(); ++i ) {
            verts->at( i ) = m_planet.points[i];
        }
    }
}

void PointsGeometry::updateColors() {
    auto* cols = colorBuffer();
    for ( int i = 0; i < m_planet.points.size(); ++i ) {
        cols->at( i ) = vl::fvec4( (m_planet.points[i] + 1.0) / 2.0 * vl::fvec3( 1.0, 0.8, 0.9), 1.0f );
    }
}
