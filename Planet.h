#pragma once

#include <vector>
#include <vlCore/Vector3.hpp>
#include <set>

#include "util/fmath.h"
#include "util/Profiler.h"

class Planet {
public:
    Planet();

    void generate( int n, float jitter, bool centroid, bool normalize, int plates, float collisionThreshold, float moisture );

    struct edge {
        std::size_t a;
        std::size_t b;
        std::size_t neighbor;
        float distance;
        float length;
        float bearing;
        bool plateBorder;
    };

    struct cell {
        std::size_t point;
        std::vector<edge> edges;
        std::size_t plate;
        vl::fvec3 color;
        vl::fvec3 dir;
        vl::fvec3 wind;
        vl::fvec3 incomingWind;
        float elevation;
        float moisture;
        float compression;
        float dMnt;
        float dCst;
        float dOcn;
        float illumination;
        float annualIllumination;
        float temperature;
        std::size_t r;
    };

    struct plate {
        vl::vec3 origin;
        std::size_t cell;
        vl::fvec3 axisOfRotation;
        float elevation;
    };

    std::vector<vl::fvec3> points;
    std::vector<vl::dvec2> coords;
    std::vector<vl::fvec3> centers;
    std::vector<std::size_t> triangles;
    std::vector<std::size_t> halfedges;
    std::vector<cell> cells;
    std::vector<plate> plates;
    std::size_t N;
    double nScale;
    float axialTilt;

    static vl::dvec2 toStereo( const vl::fvec3& v );

    vl::fvec3 getColor( float elevation, float moisture );

    Profiler::resultset lastResults;

    void calcLight( const vl::fvec3& ray );

    void updateTemperature( float delta );

    void calcAnnualIllumination( std::size_t yearSamples, std::size_t daySamples );

    std::vector<vl::fvec2> getDailyIllumination(
            std::size_t cell,
            float timeOfYear,
            std::size_t samples );

    std::vector<vl::fvec2> getAnnualIllumination(
            std::size_t cell,
            std::size_t samples );

private:
    void generatePoints( std::vector<vl::fvec3>& pts, float jitter );

    std::vector<double> getStereoPoints();

    void fillSouthPole();

    static int nextSide( int side );
    static int prevSide( int side );

    static vl::fvec3 latLonToVec( double lat, double lon ) {
        return vl::vec3( cos( lon ) * sin( lat ),  cos( lat ), sin( lon ) * sin( lat ) );
    }
    static vl::dvec2 vecToLatLon( const vl::fvec3& v ) {
        return vl::dvec2( vecToLat( v ), vecToLon( v ) );
    }
    static double vecToLat( const vl::fvec3& v ) {
        return acos( v.y() );
    }
    static double vecToLon( const vl::fvec3& v ) {
        return atan2( v.z(), v.x() );
    }

    void generateCenters( bool centroid, bool normalize );

    void generateCells( float moisture );

    void generatePlates( int plateCount );

    void applyPlateMotion( float collisionThreshold );

    void updateCellColors();

    void updateWind();

    math::rng rnd;

    static vl::fvec3 getDir( const vl::fvec3& v, float degrees );
    static vl::fvec3 getDir( const vl::fvec3& v, const vl::fvec3& axis, float degrees );

    std::vector<float> assignDistanceField( const std::set<size_t>& seeds, const std::set<size_t>& stops );

    std::vector<vl::fvec3> colormap;

    std::vector<double> annualIllumination;

    void generateColorMap();

    void calculateCoords();

    static double safeGetAngle( const vl::fvec3& v1, const vl::fvec3& v2 );
    static double getGreatCircleDistance( const vl::fvec3& v1, const vl::fvec3& v2);
    static double getBearing( const vl::fvec3& v1, const vl::fvec3& v2);
    static double getBearing2( const vl::fvec3& v1, const vl::fvec3& v2 );
    static double angleTo( const vl::fvec3& v1, const vl::fvec3& v2, const vl::fvec3& n );
    static double sinAcos( double d );
    static vl::fvec3 destinationPoint( const vl::fvec3& v, double distance, double bearing );

    double getIllumination( const vl::fvec3& sun, const vl::fvec3& p );


    static double angleTo2( const vl::fvec3& v1, const vl::fvec3& v2, const vl::fvec3& n );

    static double getBearing3( const vl::fvec3& v1, const vl::fvec3& v2 );

    static double distanceToLine( const vl::fvec3& l, const vl::fvec3& p );
};



