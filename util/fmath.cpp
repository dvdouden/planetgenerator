#include "fmath.h"

namespace math {

    rng::rng( unsigned int seed ) :
            engine( seed ) { }

    rng& rng::operator>>( int& nr ) {
        nr = engine();
        return *this;
    }

    rng& rng::operator>>( unsigned char& nr ) {
        nr = engine();
        return *this;
    }

    int rng::operator()( int max ) {
        std::uniform_int_distribution<int> uni( 0, max );
        return uni( engine );
    }

    int rng::operator()( int min, int max ) {
        std::uniform_int_distribution<int> uni( min, max );
        return uni( engine );
    }

    float rng::operator()( float max ) {
        std::uniform_real_distribution<float> uni( 0, max );
        return uni( engine );
    }

    float rng::operator()( float min, float max ) {
        std::uniform_real_distribution<float> uni( min, max );
        return uni( engine );
    }

}
