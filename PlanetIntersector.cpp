#include "Planet.h"
#include "PlanetIntersector.h"

int PlanetIntersector::intersect( const vl::Ray& ray ) {
    vl::fvec3 v1, v2;
    if( !getIntersections( ray, v1, v2 ) ) {
        return -1;
    }

    // now that we have our intersection point, we can try to find which cell we are intersecting
    const tree* t = m_planet.kdTree.find( v1 );

    // for starters, let's find the point that's closest
    float distance = 0;
    int cell = -1;
    for ( const auto& p : t->points ) {
        if ( cell == -1 || (p.first - v1).lengthSquared() < distance ) {
            cell = p.second;
            distance = (p.first - v1).lengthSquared();
        }
    }
    return cell;
}

bool PlanetIntersector::getIntersections( const vl::Ray& ray, vl::fvec3& v1, vl::fvec3& v2 ) {
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
    vl::fvec3 L = -ray.origin();
    float tca = vl::dot(L, ray.direction() );
    if (tca < 0) {
        return false;
    }
    float d2 = vl::dot( L, L) - tca * tca;
    if (d2 > 1) {
        return false;
    }
    float thc = sqrtf(1.0f - d2);
    float t0 = tca - thc;
    float t1 = tca + thc;

    if ( t0 > t1 ) {
        std::swap( t0, t1);
    }
    if ( t0 < 0 ) {
        t0 = t1;
        if ( t0 < 0 ) {
            return false;
        }
    }
    v1 = ray.direction() * t0 + ray.origin();
    v2 = ray.direction() * t1 + ray.origin();
    return true;
}
