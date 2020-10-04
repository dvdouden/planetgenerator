#pragma once

#include <vlCore/Ray.hpp>

class Planet;

class PlanetIntersector {
public:
    PlanetIntersector( const Planet& planet ) :
    m_planet( planet ) {}

    int intersect( const vl::Ray& ray );

private:
    const Planet& m_planet;

    static bool getIntersections( const vl::Ray& ray, vl::fvec3& v1, vl::fvec3& v2 );
};



