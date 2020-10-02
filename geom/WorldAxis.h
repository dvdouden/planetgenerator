#pragma once


#include "DrawArrayGeometry.h"

class WorldAxis : public DrawArrayGeometry {
public:
    explicit WorldAxis() :
            DrawArrayGeometry( "WorldAxis", vl::PT_LINES ) {}

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

private:
};



