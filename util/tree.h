#pragma once

#include <algorithm>
#include <vector>
#include <vlCore/AABB.hpp>
#include <vlCore/Vector3.hpp>

// TODO: make it possible to construct the tree with a set of points
// TODO: make maximum leaf size configurable

struct tree {
    tree* left = nullptr;
    tree* right = nullptr;
    int axis = -1;
    float median = 0.0f;
    typedef std::pair<vl::fvec3, std::size_t> point;
    std::vector<point> points;
    vl::AABB aabb;

    ~tree() {
        delete left;
        delete right;
    }

    const tree* find( const vl::fvec3& v ) const {
        if ( axis == -1 ) {
            return this;
        }
        return ( v[axis] >= median ? right : left )->find( v );
    }

    void print( int level = 0 ) {
        for ( int i = 0; i < level; ++i ) {
            printf("-");
        }
        if ( axis == -1 ) {
            printf( "%d points\n", points.size() );
        } else {
            printf( "%d: %f\n", axis, median );
            left->print( level + 1);
            right->print( level + 1);
        }
    }

    void add( const vl::fvec3& v, std::size_t idx ) {
        if ( axis == -1 ) {
            points.push_back( std::make_pair(v, idx) );
            aabb.addPoint( v );
            if ( points.size() > 1000 ) {
                // became too large, find axis with largest difference
                axis = -1;
                if ( aabb.width() >= aabb.height() && aabb.width() >= aabb.depth() ) {
                    axis = 0;
                } else if ( aabb.height() >= aabb.width() && aabb.height() >= aabb.depth() ) {
                    axis = 1;
                } else {
                    axis = 2;
                }
                // sort points by axis
                struct axisLess {
                    axisLess( int axis ) :
                            axis( axis ) {}

                    int axis;

                    bool operator()(const point& a, const point& b) const
                    {
                        return a.first[axis] < b.first[axis];
                    }
                };

                std::sort(points.begin(), points.end(), axisLess(axis) );
                median = points[500].first[axis];
                //printf( "Split on axis %d at median %f\n", axis, median );
                left = new tree;
                right = new tree;
                for ( int i = 0; i < points.size(); ++i ) {
                    ( i < 500 ? left : right )->add( points[i].first, points[i].second );
                }
                points.clear();
            }
        } else {
            ( v[axis] >= median ? right : left )->add(v, idx);
        }
    }

    void clear() {
        delete left;
        delete right;
        left = nullptr;
        right = nullptr;
        axis = -1;
        median = 0;
        points.clear();
        aabb = vl::AABB();
    }
};
