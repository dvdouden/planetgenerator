#pragma once

// C++ port of the Java reference implementation of improved noise by Ken Perlin
// https://cs.nyu.edu/~perlin/noise/
// as described in the paper https://mrl.nyu.edu/~perlin/paper445.pdf

#include <cmath>

namespace noise {
    extern unsigned int const permutation[512];
}

template<typename T>
class PerlinNoise {
public:
    PerlinNoise() {
        for ( int i = 0; i < 256 ; i++ ) {
            p[256 + i] = p[i] = noise::permutation[i];
        }
    }

    T noise( T x, T y, T z ) {
        unsigned int X = static_cast<unsigned int>(std::floor( x )) & 255u;
        unsigned int Y = static_cast<unsigned int>(std::floor( y )) & 255u;
        unsigned int Z = static_cast<unsigned int>(std::floor( z )) & 255u;
        x -= std::floor( x );
        y -= std::floor( y );
        z -= std::floor( z );
        T u = fade( x );
        T v = fade( y );
        T w = fade( z );
        unsigned int A = p[X] + Y;
        unsigned int AA = p[A] + Z;
        unsigned int AB = p[A+1] + Z;
        unsigned int B = p[X+1]+Y;
        unsigned int BA = p[B] + Z;
        unsigned int BB = p[B+1] + Z;

        return lerp(w,
                lerp(v,
                        lerp(u,
                                grad(p[AA  ], x  , y  , z   ),
                                grad(p[BA  ], x-1, y  , z )
                                ),
                        lerp(u,
                                grad(p[AB  ], x  , y-1, z   ),
                                grad(p[BB  ], x-1, y-1, z )
                                )
                       ),
                lerp(v,
                        lerp(u,
                                grad(p[AA+1], x  , y  , z-1 ),
                                grad(p[BA+1], x-1, y  , z-1 )
                                ),
                         lerp(u,
                                 grad(p[AB+1], x  , y-1, z-1 ),
                                 grad(p[BB+1], x-1, y-1, z-1 ))));
    }

private:
    T fade( T t ) const {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    T lerp( T t, T a, T b ) const {
        return a + t * (b - a);
    }

    T grad(unsigned int hash, T x, T y, T z) {
        // convert lower four bits of hash code into 12 gradient directions
        unsigned int h = hash & 0b1111u;
        T u = h < 8 ? x : y;
        T v = h < 4 ? y : h == 12 || h == 14 ? x : z;
        return ((h & 1u) == 0 ? u : -u) + ((h & 2u) == 0 ? v : -v);
    }

    unsigned int p[512] = {0};



};



