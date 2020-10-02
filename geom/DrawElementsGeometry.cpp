#include "DrawElementsGeometry.h"

DrawElementsGeometry::DrawElementsGeometry( const std::string& name, vl::EPrimitiveType primitiveType ) {
    setObjectName( name );

    drawCalls().push_back( new vl::DrawElementsUInt( primitiveType ) );
    drawCalls()[0]->setEnabled( false );
    drawCalls()[0]->as<vl::DrawElementsUInt>()->setPrimitiveRestartEnabled( true );
}

void DrawElementsGeometry::resize( std::size_t size, std::size_t indices ) {
    if ( vertexArray() == nullptr ) {
        setVertexArray( new vl::ArrayFloat3 );
    }
    if ( colorArray() == nullptr ) {
        setColorArray( new vl::ArrayFloat4 );
    }
    vertexBuffer()->resize( size );
    colorBuffer()->resize( size );
    indexBuffer()->resize( indices );
    indexBuffer()->setBufferObjectDirty( true );
    m_colorsDirty = true;
    m_geometryDirty = true;
}

void DrawElementsGeometry::setEnabled( bool enabled ) {
    drawCalls()[0]->setEnabled( enabled );
}

bool DrawElementsGeometry::isEnabled() const {
    return drawCalls()[0]->isEnabled();
}

void DrawElementsGeometry::update() {
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

