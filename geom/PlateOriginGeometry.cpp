#include "PlateOriginGeometry.h"
#include "../Planet.h"

void PlateOriginGeometry::createGeometry() {
    resize( m_planet.plates.size() * 2 );
}

void PlateOriginGeometry::updateGeometry() {
    auto* verts = vertexBuffer();
    int idx = 0;
    for ( const auto& plate : m_planet.plates ) {
        verts->at( idx++) = plate.origin;
        verts->at( idx++) = plate.origin * 1.3f;
    }
}

void PlateOriginGeometry::updateColors() {
    auto* cols = colorBuffer();
    int idx = 0;
    for ( const auto& plate : m_planet.plates ) {
        vl::fvec4 rgba = vl::fvec4( (plate.origin + 1.0) / 2.0 * vl::fvec3( 1.0, 0.8, 0.9), 1.0f );
        cols->at(idx++) = rgba;
        cols->at(idx++) = rgba;
    }
}
