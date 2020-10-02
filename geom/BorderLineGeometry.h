#pragma once


#include "DrawArrayGeometry.h"

class Planet;

class BorderLineGeometry : public DrawArrayGeometry {
public:
    explicit BorderLineGeometry( Planet& sphere ) :
            DrawArrayGeometry( "Borderlines", vl::PT_LINES ),
            m_sphere( sphere ) {}

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

private:
    Planet& m_sphere;
    std::vector<std::size_t> m_cellIdx;
};



