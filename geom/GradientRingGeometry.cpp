#include "GradientRingGeometry.h"

void GradientRingGeometry::updateColors() {
    auto* cols = colorBuffer();
    for ( int i = 0; i < elements(); ++i ) {
        double radians = (2 * vl::dPi * i) / elements();
        vl::real col = (sin( radians ) + 1) / 2;
        cols->at( i ) = vl::fvec4( col,col,col,1);
    }
}
