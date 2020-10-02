#include "WorldAxis.h"

void WorldAxis::createGeometry() {
    resize( 6 );
}

void WorldAxis::updateGeometry() {
    auto* verts = vertexBuffer();
    verts->at( 0 ) = vl::fvec3( -1.25, 0, 0);
    verts->at( 1 ) = vl::fvec3( 2, 0, 0 );
    verts->at( 2 ) = vl::fvec3(0, -1.25, 0);
    verts->at( 3 ) = vl::fvec3( 0, 2, 0 );
    verts->at( 4 ) = vl::fvec3( 0, 0, -1.25);
    verts->at( 5 ) = vl::fvec3( 0, 0, 2 );
}

void WorldAxis::updateColors() {
    auto* cols = colorBuffer();
    cols->at( 0 ) = vl::fvec4( 1, 0, 0, 1 );
    cols->at( 1 ) = vl::fvec4( 1, 0, 0, 1 );
    cols->at( 2 ) = vl::fvec4( 0, 1, 0, 1 );
    cols->at( 3 ) = vl::fvec4( 0, 1, 0, 1 );
    cols->at( 4 ) = vl::fvec4( 0, 0, 1, 1 );
    cols->at( 5 ) = vl::fvec4( 0, 0, 1, 1 );
}
