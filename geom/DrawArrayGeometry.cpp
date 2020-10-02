#include "DrawArrayGeometry.h"

DrawArrayGeometry::DrawArrayGeometry( const std::string& name, vl::EPrimitiveType primitiveType ) {
    setObjectName( name );

    drawCalls().push_back( new vl::DrawArrays( primitiveType, 0, 0 ) );
    drawCalls()[0]->setEnabled( false );
}

void DrawArrayGeometry::resize( std::size_t size ) {
    if ( vertexArray() == nullptr ) {
        setVertexArray( new vl::ArrayFloat3 );
    }
    if ( colorArray() == nullptr ) {
        setColorArray( new vl::ArrayFloat4 );
    }
    vertexBuffer()->resize( size );
    colorBuffer()->resize( size );
    drawCalls()[0]->as<vl::DrawArrays>()->setCount( size );
    m_colorsDirty = true;
    m_geometryDirty = true;
}

void DrawArrayGeometry::setEnabled( bool enabled ) {
    drawCalls()[0]->setEnabled( enabled );
}

bool DrawArrayGeometry::isEnabled() const {
    return drawCalls()[0]->isEnabled();
}

void DrawArrayGeometry::update() {
    if ( isEnabled() ) {
        if ( m_geometryInvalid ) {
            createGeometry();
            m_geometryInvalid = false;
        }
        if ( m_geometryDirty ) {
            updateGeometry();
            vertexBuffer()->setBufferObjectDirty( true );
        }
        if ( m_colorsDirty ) {
            updateColors();
            colorBuffer()->setBufferObjectDirty( true );
        }
        if ( m_geometryDirty || m_colorsDirty ) {
            updateDirtyBufferObject( vl::BUM_KeepRamBuffer );
            m_geometryDirty = false;
            m_colorsDirty = false;
        }
    }
}

