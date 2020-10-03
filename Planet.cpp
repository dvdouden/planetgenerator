#include <vlCore/Matrix4.hpp>
#include <cmath>
#include <deque>
#include <queue>
#include "Planet.h"
#include "delaunator.h"
#include "IndexedTessellator.h"

Planet::Planet() :
rnd( 0 )
{
    generateColorMap();
}

void Planet::generate(
        int pointCount,
        float jitter,
        bool centroid,
        bool normalize,
        int plateCount,
        float collisionThreshold,
        float moisture,
        float ocean ) {

    Profiler profiler( "SphereGenerator" );
    N = pointCount;
    nScale = 2.0 / sqrt( (double)N );
    rnd = math::rng(0 );
    points.clear();
    points.resize( N );

    generatePoints( points, jitter );
    for ( int i = 0; i < points.size(); ++i ) {
        kdTree.add( points[i], i );
    }
    //kdTree.print();
    calculateCoords();
    profiler( "points" );

    std::vector<double> stereo = getStereoPoints();
    delaunator::Delaunator d( stereo );
    triangles = std::move( d.triangles );
    halfedges = std::move( d.halfedges );

    fillSouthPole();
    profiler( "triangles" );

    generateCenters( centroid, normalize );
    generateCells( moisture );
    profiler( "cells" );

    generatePlates( plateCount, ocean );
    profiler( "plates" );

    applyPlateMotion( collisionThreshold );
    profiler( "plate movement" );

    updateCellColors();
    profiler( "colors" );

    lastResults = profiler.results();
}


vl::dvec2 Planet::toStereo( const vl::fvec3& v ) {
    double y = 1.0 - v.y();
    // prevent divide by zero
    if ( y < 1e-7 ) {
        y = 1e-7;
    }
    return vl::dvec2( v.x() / y, v.z() / y );
}


std::vector<double> Planet::getStereoPoints() {
    // See <https://en.wikipedia.org/wiki/Stereographic_projection>
    std::vector<double> XY( points.size() * 2 );
    for ( int i = 0; i < points.size(); ++i ) {
        vl::dvec2 xy = toStereo( points[i] );
        XY[i * 2] = -xy.x(); // this'll flip the orientation of the generated triangles
        XY[i * 2 + 1] = xy.y();
    }
    return std::move( XY );
}


void Planet::generatePoints( std::vector<vl::fvec3>& pts, float jitter ) {
    // fibonacci sphere

    double goldenRatio = (1.0 + sqrt(5.0))/2.0;
    for ( int i = 0; i < pts.size(); ++i ) {
        double theta = 2 * vl::dPi * i / goldenRatio; // longitude
        double phi = acos(1 - 2 * (i+0.5) / pts.size() ); // latitude
        vl::vec3 v = latLonToVec( phi, theta );

        if ( jitter > 0 ) {
            // calculate a vector on the sphere that is perpendicular to V
            // just take the longitude, add 90 degrees and use latitude equator (half pi)
            /*theta += (vl::dPi / 2);
            phi = vl::dPi / 2;
            vl::vec3 v2 = latLonToVec( phi, theta );
            // rotate V2 around V by a random amount (0-360 deg)
            v2 = vl::mat4::getRotation( rnd( 360.0f ), v ) * v2;

            // now rotate V around V2
            v = vl::mat4::getRotation( rnd( jitter ) , v2 ) * v;*/

            v = destinationPoint( v, rnd( jitter ) * vl::fDEG_TO_RAD, rnd( 360.0f ) * vl::fDEG_TO_RAD );
        }

        pts[i] = v;
    }
}

void Planet::fillSouthPole() {
    // delaunator leaves a hole in the south pole that needs to be patched
    // we'll use the tessellator for that

    // start by finding all the edges that have no neighbor
    std::vector<std::pair<int, int>> pieces;
    std::vector<int> idxs;
    int firstMissing = -1;
    for ( int i = 0; i < halfedges.size(); ++i ) {
        if ( halfedges[i] == -1 ) {
            if ( firstMissing == -1 ) {
                // to speed up the lookup later on
                firstMissing = i;
            }
            int tri = i / 3;
            int a = i % 3;
            int b = (a + 1) % 3;
            // store the edge a-b
            pieces.emplace_back( triangles[i], triangles[tri * 3 + b]);
        }
    }

    // create the tessellator
    vl::IndexedTessellator tess;
    tess.setWindingRule(vl::TW_TESS_WINDING_ODD); // default
    tess.setTessNormal(vl::fvec3(0,1,0)); // default is vl::fvec3(0,0,0)
    tess.setBoundaryOnly(false); // default
    tess.setTolerance(0.0); // default
    tess.setTessellateIntoSinglePolygon(true);

    // we'll need just one shape; define its size
    tess.contours().push_back( pieces.size() );

    // figure out the contour
    int first = pieces[0].first;
    for ( int i = 0; i < pieces.size(); ++i ) {
        for (auto & piece : pieces) {
            if ( piece.first == first ) {
                tess.contourVerts().emplace_back( points[first] );
                tess.indices().emplace_back( first );
                first = piece.second;
            }
        }
    }

    // kick it off
    tess.tessellate();

    // translate result to indices
    std::vector<int> tris = tess.tessellatedTris();
    std::size_t endIdx = points.size() - 1;
    points.reserve( points.size() + tess.combinedVerts().size() );
    for ( const auto& v : tess.combinedVerts() ) {
        points.push_back( v );
        //printf( "Added new vert [%d] %f %f %f\n", points.size() - 1, v.x(), v.y(), v.z() );
    }

    triangles.reserve( triangles.size() + tris.size() );
    halfedges.reserve( halfedges.size() + tris.size() );

    for ( int tri : tris ) {
        //printf( "[%d] = %d\n", triangles.size(), tri );
        if ( tri < 0 ) {
            tri = endIdx - tri;
            //printf( "Will become %d\n", tri );
        }
        triangles.push_back( tri );
        // we'll fix the halfedges later
        halfedges.push_back( -1 );
    }

    // fix all the missing half-edges
    for ( int i = firstMissing; i < triangles.size(); ++i ) {
        if ( halfedges[i] != -1 ) {
            continue;
        }
        int nextEdge = nextSide( i );
        auto a = triangles[i];
        auto b = triangles[nextEdge];
        //printf( "[%d] looking for %d, %d\n", i, a, b);

        // find the triangle with side b-a
        for ( int j = i + 1; j < triangles.size(); ++j ) {
            if ( triangles[j] != b ) {
                continue;
            }
            int nextEdge2 = nextSide(j);
            if ( triangles[nextEdge2] == a) {
                //printf("found at %d\n", j);
                halfedges[i] = j;
                halfedges[j] = i;
                break;
            }
        }
    }
}

int Planet::nextSide( int side ) {
    return (side % 3) == 2 ? side - 2 : side + 1;
}

int Planet::prevSide( int side ) {
    return (side % 3) == 0 ? side + 2 : side - 1;
}

void Planet::generateCenters( bool centroid, bool normalize ) {
    centers.clear();
    centers.resize( triangles.size() / 3 );
    if ( centroid ) {
        for ( int i = 0; i < centers.size(); i++ ) {
            // just calculate the average of the three corners of the triangle
            centers[i] =
                    ((points[ triangles[i * 3] ] +
                    points[ triangles[i * 3 + 1] ] +
                    points[ triangles[i * 3 + 2] ] ) / 3);

            if ( normalize ) {
                centers[i] = centers[i].normalize();
            }
        }
    } else {
        for ( int i = 0; i < centers.size(); i++ ) {
            vl::fvec3 a = points[ triangles[i * 3] ];
            vl::fvec3 b = points[ triangles[i * 3 + 1] ];
            vl::fvec3 c = points[ triangles[i * 3 + 2] ];

            vl::fvec3 ac = c - a;
            vl::fvec3 ab = b - a;
            vl::fvec3 abXac =  vl::cross( ab, ac );
            vl::fvec3 numerator =
                    (vl::cross( abXac, ab ) * ac.lengthSquared()) +
                    (vl::cross( ac, abXac ) * ab.lengthSquared());
            vl::fvec3 toCircumsphereCenter = numerator * ( 1.0 / ( 2.0 * abXac.lengthSquared() ) );
            centers[i] = (a + toCircumsphereCenter);
            if ( normalize ) {
                centers[i] = centers[i].normalize();
            }
        }
    }
}

void Planet::generateCells( float moisture ) {
    cells.clear();
    cells.resize( points.size() );
    for ( int i = 0; i < points.size(); ++i ) {
        cells[i].point = i;
        cells[i].plate = -1;
        cells[i].color = (points[i] + 1.0) / 2.0 * vl::fvec3(1.0, 0.8, 0.9);
        cells[i].moisture = moisture;
    }
    for ( std::size_t i = 0; i < triangles.size(); ++i ) {
        std::size_t innerTri = i / 3;
        std::size_t outerTri = halfedges[i] / 3;
        std::size_t point = triangles[i];
        std::size_t neighbor = triangles[nextSide(i)];
        auto distance = (float)getGreatCircleDistance(points[point], points[neighbor]);
        auto length = (float)getGreatCircleDistance(centers[outerTri], centers[innerTri]);
        double bearing = getBearing3( points[point], points[neighbor] ) ;
        const auto& p = points[point];
        const auto& n = points[neighbor];
        //printf( "%f,%f,%f - %f,%f,%f = %f (%f) %f (%f)\n", p.x(), p.y(), p.z(), n.x(), n.y(), n.z(), bearing, bearing*vl::dRAD_TO_DEG, bearing2, bearing2*vl::dRAD_TO_DEG );
        cells[point].edges.push_back( {outerTri, innerTri, neighbor, distance, length, static_cast<float>(bearing)} );
    }

    // sort edges
    for ( cell& cell : cells ) {
        std::vector<edge> sortedEdges( cell.edges.size() );
        sortedEdges[0] = cell.edges[0];
        for ( int i = 1; i < cell.edges.size(); ++i ) {
            for (const auto & edge : cell.edges) {
                if ( edge.a == sortedEdges[ i - 1 ].b ) {
                    sortedEdges[i] = edge;
                    break;
                }
            }
        }
        cell.edges = std::move( sortedEdges );
    }
}

void Planet::generatePlates( int plateCount, float ocean ) {
    // reset rng
    rnd = math::rng( 0 );

    // pick random regions

    // now we could just pick any random region, but instead we're going to pick a number of random points
    // on the sphere and find the region that's closest to that point
    // this way the selected region will remain more or less the same, regardless of the number of regions

    // we'll first generate some points using the fibonacci sphere
    std::vector<vl::fvec3> pts( 10000 );
    generatePoints( pts, 0.0f );

    // then pick random points
    struct plateOrigin {
        vl::fvec3 v;
        vl::fvec3 axis;
        float distance = 0.0f;
        float rot;
        int cell = -1;
    };
    std::vector<plateOrigin> origins( plateCount );
    for ( auto& origin : origins ) {
        origin.v = pts[rnd( (int)pts.size() ) ];

        // assign a random direction by moving v in a random direction
        vl::fvec3 v = origin.v;

        // calculate a vector on the sphere that is perpendicular to V
        // just take the longitude, add 90 degrees and use latitude equator (half pi)
        double theta = vecToLon( v ) + (vl::dPi / 2);
        double phi = vl::dPi / 2;

        vl::vec3 v2 = latLonToVec( phi, theta );
        // rotate V2 around V by a random amount (0-360 deg)
        origin.rot = rnd( 360.0f );
        v2 = vl::mat4::getRotation( origin.rot, v ) * v2;

        // now rotate V around V2
        origin.axis = v2;
    }

    // now find the nearest region for each platePoint (FIXME: this slows down FAST)
    /*for ( const auto& cell : cells ) {
        for ( auto& origin : origins ) {
            if ( origin.cell == -1 || (origin.v - points[cell.point]).lengthSquared() < origin.distance ) {
                origin.cell = cell.point;
                origin.distance = (origin.v - points[cell.point]).lengthSquared();
            }
        }
    }*/
    for ( auto& origin : origins ) {
        tree* t = kdTree.find( origin.v );
        for ( const auto& p : t->points ) {
            if ( origin.cell == -1 || (origin.v - p.first).lengthSquared() < origin.distance ) {
                origin.cell = p.second;
                origin.distance = (origin.v - p.first).lengthSquared();
            }
        }
    }


    std::deque<int> todo;
    plates.clear();
    for ( auto& plate : origins ) {
        if ( plate.cell != -1 && cells[plate.cell].plate == -1 ) {
            cells[plate.cell].plate = todo.size();
            cells[plate.cell].dir = getDir( points[cells[plate.cell].point], plate.axis, 5 );
            todo.push_back( plate.cell );
            plates.push_back(
                    { plate.v,
                      static_cast<std::size_t>(plate.cell),
                      plate.axis,
                      false,
                      1 });
        }
    }

    // do a breadth first search
    rnd = math::rng( 0 );
    int largest = 0;
    while ( !todo.empty() ) {
        if ( todo.size() > largest ) {
            largest = todo.size();
        }
        cell& c = cells[todo.front()];
        todo.pop_front();
        for ( auto& edge : c.edges ) {
            cell& neighbor = cells[edge.neighbor];
            if ( neighbor.plate == -1 ) {
                neighbor.plate = c.plate;
                plates[c.plate].cellCount++;
                neighbor.elevation = c.elevation;
                // direction of cell will lower when distance to axis of rotation gets lower
                neighbor.dir = getDir(
                        points[neighbor.point],
                        plates[c.plate].axisOfRotation,
                        5 ) * distanceToLine(points[neighbor.point], plates[c.plate].axisOfRotation);
                //auto it1 = std::next(todo.begin(), rnd((int)todo.size()));
                //todo.insert( it1, edge.neighbor );
                todo.push_back( edge.neighbor );
            }
        }
    }
    for ( auto& cell : cells ) {
        for ( auto& edge : cell.edges ) {
            edge.plateBorder = cells[edge.neighbor].plate != cell.plate;
        }
    }

    std::size_t cellCount = 0;
    std::size_t oceanCount = 0;
    for( auto& plate : plates ) {
        if ( float(oceanCount + plate.cellCount) / cells.size() <= ocean ) {
            plate.oceanic = true;
            oceanCount += plate.cellCount;
        }
        cellCount += plate.cellCount;
    }
    printf( "Assigned %d of %d cells to %d plates (%d oceanic, %d%%)\n",
            cellCount,
            cells.size(),
            plates.size(),
            oceanCount,
            (oceanCount * 100) / cellCount );

}

vl::fvec3 Planet::getDir( const vl::fvec3& v, float degrees ) {
    // assign a random direction by moving v in a random direction
    // calculate a vector on the sphere that is perpendicular to V
    // just take the longitude, add 90 degrees and use latitude equator (half pi)
    double theta = vecToLon( v ) + (vl::dPi / 2);
    double phi = vl::dPi / 2;

    vl::vec3 v2 = latLonToVec( phi, theta );
    v2 = vl::mat4::getRotation( degrees, v ) * v2;

    // now rotate V around V2
    vl::vec3 v3 = vl::mat4::getRotation( 5, v2 ) * v;
    return (v3 - v).normalize();
}

vl::fvec3 Planet::getDir( const vl::fvec3& v, const vl::fvec3& axis, float degrees ) {
    return ((vl::mat4::getRotation( degrees, axis ) * v) - v).normalize();
}

void Planet::updateCellColors() {
    for ( auto& cell : cells ) {
        cell.color = getColor( cell.elevation, cell.moisture );
    }
}

vl::fvec3 Planet::getColor( float elevation, float moisture ) {
    float e = elevation > 0.0f? 0.5f * (elevation * elevation + 1.0f) : 0.5f * (elevation + 1.0f);
    // TODO: add moisture
    int eIdx = e * 63.0f;
    int mIdx = moisture * 63.0f;

    return colormap[mIdx * 64 + eIdx];
}

void Planet::applyPlateMotion( float collisionThreshold ) {
    float epsilon = 1e-2f;
    std::set<std::size_t> coastlines;
    std::set<std::size_t> mountains;
    std::set<std::size_t> oceans;
    // FIXME: something weird is going on here, doesn't work correctly...
    for ( auto& c : cells ) {
        float bestCompression = 0;//std::numeric_limits<float>::infinity();
        int r = -1;
        for ( auto& edge : c.edges ) {
            const cell& n = cells[edge.neighbor];
            if ( n.plate == c.plate ) {
                // only interested in border cells
                continue;
            }
            float distance = (points[c.point] - points[n.point]).length();
            float distance2 = ((points[c.point] + (c.dir * nScale)) - (points[n.point] + (n.dir * nScale))).length();
            float compression = distance - distance2;
            if ( r == -1 || compression > bestCompression ) {
                r = edge.neighbor;
                bestCompression = compression;
                c.compression = bestCompression / nScale;
                c.r = r;
            }
        }
        if ( r != -1) {
            bool collided = bestCompression > collisionThreshold * nScale;
            if ( plates[c.plate].oceanic && plates[cells[r].plate].oceanic ) {
                (collided ? coastlines : oceans).insert( c.point );
            } else if ( !plates[c.plate].oceanic && !plates[cells[r].plate].oceanic ) {
                if ( collided ) {
                    mountains.insert( c.point );
                }
            } else {
                (collided ? mountains : coastlines ).insert( c.point);
            }
        }
    }

    for ( auto& plate : plates ) {
        ((plate.oceanic) ? oceans : coastlines ).insert( plate.cell );
    }

    std::set<std::size_t> stopCells;
    stopCells.insert( coastlines.begin(), coastlines.end() );
    stopCells.insert( mountains.begin(), mountains.end() );
    stopCells.insert( oceans.begin(), oceans.end() );

    std::vector<float> distMountains = assignDistanceField( mountains, stopCells );
    std::vector<float> distOceans = assignDistanceField( oceans, stopCells );
    std::vector<float> distCoastlines = assignDistanceField( coastlines, stopCells );

    epsilon = 1e-3;
    for ( auto& cell : cells ) {
        cell.dMnt = distMountains[cell.point] + epsilon;
        cell.dOcn = distOceans[cell.point] + epsilon;
        cell.dCst = distCoastlines[cell.point] + epsilon;
        if ( cell.dMnt == std::numeric_limits<float>::infinity() && cell.dOcn == std::numeric_limits<float>::infinity() ) {
            cell.elevation = 0.1;
        } else {
            cell.elevation = (float)((1.0 / cell.dMnt - 1.0 / cell.dOcn) / (1.0 / cell.dMnt + 1.0 / cell.dOcn + 1.0 / cell.dCst));
        }
    }
}

std::vector<float> Planet::assignDistanceField( const std::set<size_t>& seeds, const std::set<size_t>& stops ) {
    std::vector<float> distances( cells.size(), std::numeric_limits<float>::infinity() );
    rnd = math::rng( 0 );

    std::vector<std::size_t> queue;
    queue.reserve( cells.size() );
    for ( std::size_t seed : seeds ) {
        queue.push_back( seed );
        distances[seed] = 0;
    }

    // breadth first search
    for ( std::size_t i = 0; i < queue.size(); ++i ) {
        std::size_t pos = i + ((i == queue.size() - 1) ? 0 : rnd( ((int)(queue.size() - i)) - 1 ));
        std::size_t cIdx = queue[pos];
        queue[pos] = queue[i];
        cell& c = cells[cIdx];
        for ( const auto& edge : c.edges ) {
            if ( distances[edge.neighbor] == std::numeric_limits<float>::infinity() && stops.find(edge.neighbor) == stops.end() ) {
                distances[edge.neighbor] = distances[c.point] + nScale;
                queue.push_back( edge.neighbor );
            }
        }
    }

    return std::move( distances );
}

void Planet::generateColorMap() {
    int width = 64;
    int height = 64;
    colormap.resize( width * height );

    for (int y = 0, p = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float e = 2.0f * x / width - 1,
                    m = (float)y / height;

            int r, g, b;

            if (x == width/2 - 1) {
                r = 48;
                g = 120;
                b = 160;
            } else
            if (x == width/2 - 2) {
                r = 48;
                g = 100;
                b = 150;
            } else if (x == width/2 - 3) {
                r = 48;
                g = 80;
                b = 140;
            } else
            if (e < 0.0) {
                r = 48 + 48*e;
                g = 64 + 64*e;
                b = 127 + 127*e;
            } else { // adapted from terrain-from-noise article
                m = m * (1-e); // higher elevation holds less moisture; TODO: should be based on slope, not elevation

                r = 210 - 100*m;
                g = 185 - 45*m;
                b = 139 - 45*m;
                r = 255 * e + r * (1-e),
                g = 255 * e + g * (1-e),
                b = 255 * e + b * (1-e);
            }

            colormap[p++] = vl::fvec3( r / 255.0f, g/255.0f, b/255.0f );
        }
    }
}

void Planet::calcLight( const vl::fvec3& ray ) {
    for ( auto& cell : cells ) {
        cell.illumination = (float)getIllumination( ray, points[cell.point] );
    }
}

std::vector<vl::fvec2> Planet::getDailyIllumination(
        std::size_t cell,
        float timeOfYear,
        std::size_t samples) {
    std::vector<vl::fvec2> illumination( samples );

    double radTOY = (timeOfYear + 0.5) * 2 * vl::dPi; // offset by half to start in winter for northern hemisphere
    double sunLatitude = axialTilt * cos( radTOY );
    double cosLat = cos( sunLatitude * vl::dDEG_TO_RAD );
    double sinLat = sin( sunLatitude * vl::dDEG_TO_RAD );
    vl::fvec3 p = points[cells[cell].point];
    for ( std::size_t i = 0; i < samples; ++i ) {
        double timeOfDay = (double)i / samples;
        double radians = (2 * vl::dPi * timeOfDay); // negate to reverse rotation of sun
        vl::fvec3 sun(
                cosLat * cos( radians ),
                sinLat,
                cosLat * sin( radians ) );
        illumination[i] = vl::fvec2( timeOfDay, getIllumination( sun, p ) );
    }

    return illumination;
}


std::vector<vl::fvec2> Planet::getAnnualIllumination(
        std::size_t cell,
        std::size_t samples) {
    std::vector<vl::fvec2> illumination( samples );

    for ( std::size_t i = 0; i < samples; ++i ) {
        double timeOfYear = (double)i / samples;
        double radTOY = (timeOfYear + 0.5) * 2 * vl::dPi; // offset by half to start in winter for northern hemisphere
        double sunLatitude = axialTilt * cos( radTOY );
        double cosLat = cos( sunLatitude * vl::dDEG_TO_RAD );
        double sinLat = sin( sunLatitude * vl::dDEG_TO_RAD );

        double tmpIllumination = 0;
        vl::fvec3 p = points[cells[cell].point];
        for ( std::size_t j = 0; j < 10; ++j ) {
            double timeOfDay = (double)j / 10;
            double radians = (2 * vl::dPi * timeOfDay); // negate to reverse rotation of sun
            vl::fvec3 sun(
                    cosLat * cos( radians ),
                    sinLat,
                    cosLat * sin( radians ) );
            tmpIllumination += getIllumination( sun, p );
        }
        illumination[i] = vl::fvec2( timeOfYear, tmpIllumination / 10.0 );
    }

    return illumination;
}

void Planet::calcAnnualIllumination( std::size_t yearSamples, std::size_t daySamples ) {
    annualIllumination.clear();
    annualIllumination.resize( 181, 0 );

    // calculate the annual illumination for every degree of latitude
    double oneOverSamples = 1.0 / (yearSamples * daySamples);
    for ( std::size_t i = 0; i < yearSamples; ++i ) {
        float timeOfYear = (float)i / yearSamples;
        double radTOY = (timeOfYear + 0.5) * 2 * vl::dPi;
        double sunLatitude = axialTilt * cos( radTOY );
        double cosLat = cos( sunLatitude * vl::dDEG_TO_RAD );
        double sinLat = sin( sunLatitude * vl::dDEG_TO_RAD );
        for ( std::size_t j = 0; j < daySamples; ++j ) {
            double timeOfDay = (double)j / daySamples;
            double radians = (2 * vl::dPi * timeOfDay); // negate to reverse rotation of sun
            vl::fvec3 sun(
                    cosLat * cos( radians ),
                    sinLat,
                    cosLat * sin( radians ) );

            for ( int lat = 0; lat <= 180; ++lat ) {
                annualIllumination[lat] +=
                        getIllumination( sun, latLonToVec( lat * vl::dDEG_TO_RAD, 0 ) ) * oneOverSamples;
            }
        }
    }

    // now use linear interpolation to update the individual cells (good enough)
    for ( auto& cell : cells ) {
        double lat = coords[cell.point].x() * vl::dRAD_TO_DEG;
        double lowLat;
        double highFact = std::modf( lat, &lowLat );
        double lowFact = 1.0 - highFact;
        double highLat = lowLat + 1;
        if ( highLat > 180 ) {
            highLat = 180;
        }

        cell.annualIllumination = (float)(annualIllumination[lowLat] * lowFact + annualIllumination[highLat] * highFact);
    }

}

void Planet::calculateCoords() {
    coords.resize( points.size() );
    for ( int i = 0; i < coords.size(); ++i ) {
        coords[i] = vecToLatLon( points[i] );
    }
}

double Planet::safeGetAngle( const vl::fvec3& v1, const vl::fvec3& v2 ) {
    double dot = vl::dot( v1, v2 );
    // in some occasions, the result can exceed the range [-1,1]; clamp the result
    if ( dot > 1.0 ) { dot = 1.0; }
    if ( dot < -1.0 ) { dot = -1.0; }
    return std::acos( dot );
}

double Planet::getGreatCircleDistance( const vl::fvec3& v1, const vl::fvec3& v2 ) {
    // https://en.wikipedia.org/wiki/Great-circle_distance
    // calculates the shortest distance on sphere given two points on a unit-sphere
    // it can be calculated by taking the angle between the two points
    // express it as the percentage of a circle
    // multiply by the circumference of the circle, and done
    // In our case, we have a unit-sphere, so the circumference is 2PI
    // And the angle is expressed in radians, so that's also 2PI
    // So our distance is equal to our angle!
    return safeGetAngle( v1, v2 );
}


double Planet::getBearing( const vl::fvec3& v1, const vl::fvec3& v2 ) {
    // See https://www.askamathematician.com/2018/07/q-given-two-points-on-the-globe-how-do-you-figure-out-the-direction-and-distance-to-each-other/
    //
    // first we need the distance between v1 and v2
    // this could be calculated using the law of cosines
    // cos(c) = cos(a) * cos(b) + sin(a) * sin(b) * cos(C)
    // where c is the distance between v1 and v2
    // a = latitude of v1
    // b = latitude of v2
    // C = difference in longitude between v1 and v2
    // so
    // c = acos( cos(a) * cos(b) + sin(a) * sin(b) * cos(C) )
    // it should be noted that the latitude of v is acos v.y, so:
    // cos(a) = cos( acos( v1.y ) ) = v1.y
    // cos(b) = cos( acos( v2.y ) ) = v2.y
    // which leads to
    // c = acos( v1.y * v2.y + sin(a) * sin(b) * cos(C) )
    // furthermore, sin( acos( X ) ) == sqrt( 1 - X*X )
    //
    // anyway, long story short
    // c = acos( dot( v1, v2 ) )
    // so
    // cos(c) = dot( v1, v2 )
    //
    // now to get our bearing B
    // cos(b) = cos(a) * cos(c) + sin(a) * sin(c) * cos(B)
    // which leads to
    // B = acos( (cos(b) - cos(a) * cos(c)) / (sin(a) * sin(c)) )
    // but since cos(b) == v2.y and cos(a) == v1.y
    // and cos(c) == cos(acos(dot(v1,v2))) == dot(v1, v2)
    // we can write:
    // B = acos( (v2.y - v1.y * dot(v1,v2)) / (sin(acos(v1.y)) * sin(acos(dot(v1,v2)))) )
    //
    // sin(acos(v1.y)) can be written as sqrt( 1 - v1.y * v1.y )
    // and
    // sin(acos(dot(v1,v2))) as sqrt(1 - dot(v1,v2) * dot(v1,v2))
    //
    // so
    // B = acos( (v2.y - v1.y * dot(v1,v2)) / (sqrt( 1 - v1.y * v1.y ) * sqrt(1 - dot(v1,v2) * dot(v1,v2))) )
    double d = vl::dot(v1, v2);
    double bearing = acos( (v2.y() - v1.y() * d) / (sinAcos(v1.y()) * sinAcos(d)) );
    return bearing;
}

double Planet::getBearing2( const vl::fvec3& v1, const vl::fvec3& v2 ) {
    // more elegant than getBearing, but slightly slower
    // though it does return the correct sign...
    // https://www.movable-type.co.uk/scripts/latlong-vectors.html
    vl::fvec3 n( 0, 1, 0); // north pole
    vl::fvec3 c1 = vl::cross( v1, v2 ); // great circle through v1 and v2
    vl::fvec3 c2 = vl::cross( v1, n ); // great circle through v1 and north pole

    return angleTo(c1, c2, v1); // bearing is (signed) angle between c1 & c2
}

double Planet::getBearing3( const vl::fvec3& v1, const vl::fvec3& v2 ) {
    // more elegant than getBearing, but slightly slower
    // though it does return the correct sign...
    // https://www.movable-type.co.uk/scripts/latlong-vectors.html
    vl::fvec3 n( 0, 1, 0); // north pole
    vl::fvec3 c1 = vl::cross( v1, v2 ); // great circle through v1 and v2
    vl::fvec3 c2 = vl::cross( v1, n ); // great circle through v1 and north pole

    return angleTo2(c1, c2, v1); // bearing is (signed) angle between c1 & c2
}

double Planet::angleTo( const vl::fvec3& v1, const vl::fvec3& v2, const vl::fvec3& n ) {
    // q.v. stackoverflow.com/questions/14066933#answer-16544330, but n·p₁×p₂ is numerically
    // ill-conditioned, so just calculate sign to apply to |p₁×p₂|

    // if n·p₁×p₂ is -ve, negate |p₁×p₂|
    vl::fvec3 cp = vl::cross(v1, v2);
    double sign = vl::dot(cp, n) >= 0 ? 1 : -1;

    double det = cp.length() * sign;
    double dot = vl::dot( v1, v2);

    return atan2( det, dot );
}

double Planet::angleTo2( const vl::fvec3& v1, const vl::fvec3& v2, const vl::fvec3& n ) {
    // https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors#answer-16544330
    double dot = vl::dot( v1, v2 );

    // https://en.wikipedia.org/wiki/Determinant
    double det =
            v1.x()*v2.y()*n.z() + v2.x()*n.y()*v1.z() + n.x()*v1.y()*v2.z() -
            v1.z()*v2.y()*n.x() - v2.z()*n.y()*v1.x() - n.z()*v1.y()*v2.x();
    return atan2( det, dot );
}

double Planet::sinAcos( double d ) {
    return sqrt( 1.0 - d * d );
}

vl::fvec3 Planet::destinationPoint( const vl::fvec3& a, double distance, double bearing ) {
    // https://www.movable-type.co.uk/scripts/latlong-vectors.html

    vl::fvec3 n(0, 1, 0);     // north pole

    vl::fvec3 de = vl::cross(n, a).normalize();  // east direction vector @ a
    vl::fvec3 dn = vl::cross(a, de);             // north direction vector @ a

    vl::fvec3 deSinBearing = de * sin(bearing);
    vl::fvec3 dnCosBearing = dn * cos(bearing);

    vl::fvec3 d = dnCosBearing + deSinBearing; // direction vector @ a (≡ C×a; C = great circle)

    vl::fvec3 x = a * cos(distance); // component of b parallel to a
    vl::fvec3 y = d * sin(distance); // component of b perpendicular to a

    vl::fvec3 b = x + y;

    return b;
}


double Planet::getIllumination( const vl::fvec3& sun, const vl::fvec3& p ) {
    /*
    double angle = safeGetAngle( sun, p );
    if ( angle > (vl::dPi / 2) || angle < -(vl::dPi / 2) ) {
        angle = vl::dPi / 2;
    }
    return cos( angle );*/

    // well this is pretty ridiculous, angle is arccosine of the dot product
    // illumination is the cosine of the angle
    // yes, illumination is the cosine of the arccosine of the dot product
    // ...
    // illumination is the dot product.
    double illumination = vl::dot( sun, p );
    return illumination > 0 ? illumination : 0;
}

void Planet::updateWind() {

}

void Planet::updateTemperature( float delta ) {
    // this is overly simplified
    // TODO: take difference in land/water into account
    // TODO: take air temperature into account
    // TODO: take albedo (reflectiveness) into account
    // TODO: take clouds and rain into account
    // TODO: take altitude into account
    for ( auto& cell : cells ) {
        if ( cell.illumination > 0.01 ) {
            cell.temperature += delta * cell.illumination;
        } else {
            cell.temperature -= delta * 0.5f;
        }
    }
}

double Planet::distanceToLine( const vl::fvec3& l, const vl::fvec3& p ) {
    // https://stackoverflow.com/a/52792014
    // we can simplify this by a LOT since our line always runs through the origin
    // and both l and p are already normalized
    // calculate point x on line l that's closest to p
    vl::fvec3 x = l * vl::dot( p, l );
    // get distance between x and p
    return (p - x).length();
}



