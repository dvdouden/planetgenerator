#pragma once


#include "DrawArrayGeometry.h"

class Planet;

class RingGeometry : public DrawArrayGeometry {
public:
    explicit RingGeometry() :
            DrawArrayGeometry( "Ring", vl::PT_LINE_LOOP ) {}

    void setLatitude( float latitude ) {
        m_latitude = latitude;
        markGeometryDirty();
    }

    float latitude() const {
        return m_latitude;
    }

    void setElements( int elements ) {
        m_elements = elements;
        markGeometryDirty();
    }

    int elements() const {
        return m_elements;
    }

    void setOffset( float offset ) {
        m_offset = offset;
        markGeometryDirty();
    }

    float offset() const {
        return m_offset;
    }

    void createGeometry() override;

    void updateGeometry() override;

    void updateColors() override;

private:
    float m_latitude = 0;
    int m_elements = 360;
    float m_offset = 1.02f;
};



