/**************************************************************************************/
/*                                                                                    */
/*  Visualization Library                                                             */
/*  http://visualizationlibrary.org                                                   */
/*                                                                                    */
/*  Copyright (c) 2005-2020, Michele Bosi                                             */
/*  All rights reserved.                                                              */
/*                                                                                    */
/*  Redistribution and use in source and binary forms, with or without modification,  */
/*  are permitted provided that the following conditions are met:                     */
/*                                                                                    */
/*  - Redistributions of source code must retain the above copyright notice, this     */
/*  list of conditions and the following disclaimer.                                  */
/*                                                                                    */
/*  - Redistributions in binary form must reproduce the above copyright notice, this  */
/*  list of conditions and the following disclaimer in the documentation and/or       */
/*  other materials provided with the distribution.                                   */
/*                                                                                    */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND   */
/*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED     */
/*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE            */
/*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR  */
/*  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    */
/*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      */
/*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON    */
/*  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT           */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS     */
/*  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                      */
/*                                                                                    */
/**************************************************************************************/

#include <vlGraphics/OpenGLContext.hpp>
#include <vlGraphics/Actor.hpp>
#include <vlCore/Log.hpp>

#include "Graph.h"


using namespace vl;

//-----------------------------------------------------------------------------
void Graph::render_Implementation(const Actor* actor, const Shader*, const Camera* camera, OpenGLContext* gl_context) const
{
    gl_context->bindVAS(NULL, false, false);

    // Lighting can be enabled or disabled.
    // glDisable(GL_LIGHTING);

    // Blending must be enabled explicity by the vl::Shader, also to perform z-sort.
    // glEnable(GL_BLEND);

    // Trucchetto che usiamo per evitare z-fighting:
    // Pass #1 - fill color and stencil
    // - disable depth write mask
    // - depth test can be enabled or not by the user
    // - depth func can be choosen by the user
    // - render in the order: background, border, shadow, outline, text
    // Pass #2 - fill z-buffer
    // - enable depth write mask
    // - disable color mask
    // - disable stencil
    // - drawing background and border

    // Pass #1

    // disable z-writing
    GLboolean depth_mask=0;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depth_mask);
    glDepthMask(GL_FALSE);

    // background
    if (backgroundEnabled())
        renderBackground( actor, camera );

    // border
    if (borderEnabled())
        renderBorder( actor, camera );

    // to have the most correct results we should render the text twice one for color and stencil, the other for the z-buffer

    // shadow render
    if (shadowEnabled())
        renderGraph( actor, camera, shadowColor(), shadowVector() );
    // outline render
    if (outlineEnabled())
    {
        renderGraph( actor, camera, outlineColor(), fvec2(-1,0) );
        renderGraph( actor, camera, outlineColor(), fvec2(+1,0) );
        renderGraph( actor, camera, outlineColor(), fvec2(0,-1) );
        renderGraph( actor, camera, outlineColor(), fvec2(0,+1) );
    }
    // text render
    renderGraph( actor, camera, color(), fvec2(0,0) );

    // Pass #2
    // fills the z-buffer (not the stencil buffer): approximated to the text bbox

    // restores depth mask
    glDepthMask(depth_mask);

    if (depth_mask)
    {
        // disables writing to the color buffer
        GLboolean color_mask[4];
        glGetBooleanv(GL_COLOR_WRITEMASK, color_mask);
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

        // disable writing to the stencil buffer
        int stencil_front_mask=0;
        glGetIntegerv(GL_STENCIL_WRITEMASK, &stencil_front_mask);
        int stencil_back_mask=0;
        if (Has_GL_Version_2_0)
            glGetIntegerv(GL_STENCIL_BACK_WRITEMASK, &stencil_back_mask);
        glStencilMask(0);

        // background
        renderBackground( actor, camera );

        // border
        renderBorder( actor, camera );

        // restores color writing
        glColorMask(color_mask[0],color_mask[1],color_mask[2],color_mask[3]);

        // restore the stencil masks
        glStencilMask(stencil_front_mask);
        if (Has_GL_Version_2_0)
            glStencilMaskSeparate(GL_BACK, stencil_back_mask);
    }

    // restore the right color and normal since we changed them
    glColor4fv( gl_context->color().ptr() );
    glNormal3fv( gl_context->normal().ptr() );
}
//-----------------------------------------------------------------------------
void Graph::renderGraph(const Actor* actor, const Camera* camera, const fvec4& color, const fvec2& offset) const
{
    if ( mData.empty() ) {
        return;
    }
    int viewport[] = { camera->viewport()->x(), camera->viewport()->y(), camera->viewport()->width(), camera->viewport()->height() };

    if (viewport[2] < 1) viewport[2] = 1;
    if (viewport[3] < 1) viewport[3] = 1;

    // note that we only save and restore the server side states

    if (mode() == Graph2D)
    {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        VL_CHECK_OGL();

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        // glLoadIdentity();
        // gluOrtho2D( -0.5f, viewport[2]-0.5f, -0.5f, viewport[3]-0.5f );

        // clever trick part #1
        fmat4 mat = fmat4::getOrtho(-0.5f, viewport[2]-0.5f, -0.5f, viewport[3]-0.5f, -1, +1);
        mat.e(2,2) = 1.0f; // preserve the z value from the incoming vertex.
        mat.e(2,3) = 0.0f;
        glLoadMatrixf(mat.ptr());

        VL_CHECK_OGL();
    }


    AABB rbbox = rawboundingRect(); // for text alignment
    VL_CHECK(rbbox.maxCorner().z() == 0)
    VL_CHECK(rbbox.minCorner().z() == 0)
    AABB bbox = rbbox;
    int applied_margin = backgroundEnabled() || borderEnabled() ? margin() : 0;
    bbox.setMaxCorner( bbox.maxCorner() + vec3(2.0f*applied_margin,2.0f*applied_margin,0) );
    VL_CHECK(bbox.maxCorner().z() == 0)
    VL_CHECK(bbox.minCorner().z() == 0)

    // basic render states

    fvec2 pen(0,0);


    // Constant color
    glColor4f( color.r(), color.g(), color.b(), color.a() );

    // Constant normal
    glNormal3f( 0, 0, 1 );

    auto vectSize = mData.size() + (mMarkerEnabled ? 2 : 0);
    fvec3 vect[vectSize];
    glEnableClientState( GL_VERTEX_ARRAY );
    glVertexPointer(3, GL_FLOAT, 0, vect[0].ptr());

    // viewport alignment
    fmat4 m = mMatrix;

    int w = camera->viewport()->width();
    int h = camera->viewport()->height();

    if (w < 1) w = 1;
    if (h < 1) h = 1;

    if ( !(actor && actor->transform()) && mode() == Graph2D )
    {
        if (viewportAlignment() & AlignHCenter)
        {
            VL_CHECK( !(viewportAlignment() & AlignRight) )
            VL_CHECK( !(viewportAlignment() & AlignLeft) )
            // vect[i].x() += int((viewport[2]-1.0f) / 2.0f);
            m.translate( (float)int((w-1.0f) / 2.0f), 0, 0);
        }

        if (viewportAlignment() & AlignRight)
        {
            VL_CHECK( !(viewportAlignment() & AlignHCenter) )
            VL_CHECK( !(viewportAlignment() & AlignLeft) )
            // vect[i].x() += int(viewport[2]-1.0f);
            m.translate( (float)int(w-1.0f), 0, 0);
        }

        if (viewportAlignment() & AlignTop)
        {
            VL_CHECK( !(viewportAlignment() & AlignBottom) )
            VL_CHECK( !(viewportAlignment() & AlignVCenter) )
            // vect[i].y() += int(viewport[3]-1.0f);
            m.translate( 0, (float)int(h-1.0f), 0);
        }

        if (viewportAlignment() & AlignVCenter)
        {
            VL_CHECK( !(viewportAlignment() & AlignTop) )
            VL_CHECK( !(viewportAlignment() & AlignBottom) )
            // vect[i].y() += int((viewport[3]-1.0f) / 2.0f);
            m.translate( 0, (float)int((h-1.0f) / 2.0f), 0);
        }
    }

    for ( int i = 0; i < mData.size(); ++i ) {
        vect[i].x() = mData[i].x();
        vect[i].y() = mData[i].y();
    }
    if ( mMarkerEnabled) {
        vect[vectSize - 2].x() = mMarkerPosition;
        vect[vectSize - 2].y() = mMin.y();
        vect[vectSize - 1].x() = mMarkerPosition;
        vect[vectSize - 1].y() = mMax.y();
    }

    for ( int i = 0; i < vectSize; ++i ) {
        vect[i].x() = ((vect[i].x() - mMin.x()) / (mMax.x() - mMin.x()) ) * mWidth;
        vect[i].y() = ((vect[i].y() - mMin.y()) / (mMax.y() - mMin.y()) ) * mHeight;
        vect[i].z() = 0;

        // normalize coordinate orgin to the bottom/left corner
        vect[i] -= (fvec3)bbox.minCorner();

        vect[i].x() += applied_margin;
        vect[i].y() += applied_margin;

        // apply offset for outline rendering
        vect[i].x() += offset.x();
        vect[i].y() += offset.y();

        if (alignment() & AlignHCenter)
        {
            VL_CHECK( !(alignment() & AlignRight) )
            VL_CHECK( !(alignment() & AlignLeft) )
            vect[i].x() -= (int)(bbox.width() / 2.0f);
        }

        if (alignment() & AlignRight)
        {
            VL_CHECK( !(alignment() & AlignHCenter) )
            VL_CHECK( !(alignment() & AlignLeft) )
            vect[i].x() -= (int)bbox.width();
        }

        if (alignment() & AlignTop)
        {
            VL_CHECK( !(alignment() & AlignBottom) )
            VL_CHECK( !(alignment() & AlignVCenter) )
            vect[i].y() -= (int)bbox.height();
        }

        if (alignment() & AlignVCenter)
        {
            VL_CHECK( !(alignment() & AlignTop) )
            VL_CHECK( !(alignment() & AlignBottom) )
            vect[i].y() -= int(bbox.height() / 2.0);
        }
        // apply transform
        vect[i] = m * vect[i];
        //printf("[%d] %f, %f\n", i, vect[i].x(), vect[i].y() );
    }
    // actor's transform following in Graph2D
    if ( actor->transform() && mode() == Graph2D )
    {
        vec4 v(0,0,0,1);
        v = actor->transform()->worldMatrix() * v;

        camera->project(v,v);

        // from screen space to viewport space
        v.x() -= viewport[0];
        v.y() -= viewport[1];

        v.x() = (float)int(v.x());
        v.y() = (float)int(v.y());

        // clever trick part #2
        float vZ = float((v.z() - 0.5f) / 0.5f);
        for ( int i = 0; i < mData.size(); ++i ) {
            vect[i].x() += (float)v.x();
            vect[i].y() += (float)v.y();
            vect[i].z() = vZ;
        }
    }
    glDrawArrays( GL_LINE_STRIP, 0, mData.size() ); VL_CHECK_OGL();
    if ( mMarkerEnabled ) {
        glDrawArrays( GL_LINES, mData.size(), 2 ); VL_CHECK_OGL();
    }

    VL_CHECK_OGL();

    if (mode() == Graph2D)
    {
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix(); VL_CHECK_OGL()

        glMatrixMode(GL_PROJECTION);
        glPopMatrix(); VL_CHECK_OGL()
    }

}
//-----------------------------------------------------------------------------
// returns the raw bounding box of the string, i.e. without alignment, margin and matrix transform.
AABB Graph::rawboundingRect() const
{
    AABB aabb( vl::fvec3(), vl::fvec3( mWidth, mHeight, 0 ));

    return aabb;
}
//-----------------------------------------------------------------------------
void Graph::renderBackground(const Actor* actor, const Camera* camera) const
{
    int viewport[] = { camera->viewport()->x(), camera->viewport()->y(), camera->viewport()->width(), camera->viewport()->height() };

    if (viewport[2] < 1) viewport[2] = 1;
    if (viewport[3] < 1) viewport[3] = 1;

    if (mode() == Graph2D)
    {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        VL_CHECK_OGL();

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        //glLoadIdentity();
        //gluOrtho2D( -0.5f, viewport[2]-0.5f, -0.5f, viewport[3]-0.5f );

        // clever trick part #1
        fmat4 mat = fmat4::getOrtho(-0.5f, viewport[2]-0.5f, -0.5f, viewport[3]-0.5f, -1, +1);
        mat.e(2,2) = 1.0f;
        mat.e(2,3) = 0.0f;
        glLoadMatrixf(mat.ptr());
        VL_CHECK_OGL();
    }

    // Constant color
    glColor4f(mBackgroundColor.r(),mBackgroundColor.g(), mBackgroundColor.b(), mBackgroundColor.a()); VL_CHECK_OGL()

    // Constant normal
    glNormal3f(0, 0, 1); VL_CHECK_OGL()

    vec3 a, b, c, d;
    boundingRectTransformed( a, b, c, d, camera, mode() == Graph2D ? actor : NULL );
    fvec3 vect[] = { (fvec3)a, (fvec3)b, (fvec3)c, (fvec3)d };
    glEnableClientState( GL_VERTEX_ARRAY ); VL_CHECK_OGL()
    glVertexPointer(3, GL_FLOAT, 0, vect); VL_CHECK_OGL()

    glDrawArrays(GL_TRIANGLE_FAN, 0, 4); VL_CHECK_OGL()

    glDisableClientState( GL_VERTEX_ARRAY ); VL_CHECK_OGL()

    if (mode() == Graph2D)
    {
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix(); VL_CHECK_OGL()

        glMatrixMode(GL_PROJECTION);
        glPopMatrix(); VL_CHECK_OGL()
    }
}
//-----------------------------------------------------------------------------
void Graph::renderBorder(const Actor* actor, const Camera* camera) const
{
    int viewport[] = { camera->viewport()->x(), camera->viewport()->y(), camera->viewport()->width(), camera->viewport()->height() };

    if (viewport[2] < 1) viewport[2] = 1;
    if (viewport[3] < 1) viewport[3] = 1;

    if (mode() == Graph2D)
    {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();
        VL_CHECK_OGL();

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        //glLoadIdentity();
        //gluOrtho2D( -0.5f, viewport[2]-0.5f, -0.5f, viewport[3]-0.5f );

        // clever trick part #1
        fmat4 mat = fmat4::getOrtho(-0.5f, viewport[2]-0.5f, -0.5f, viewport[3]-0.5f, -1, +1);
        mat.e(2,2) = 1.0f;
        mat.e(2,3) = 0.0f;
        glLoadMatrixf(mat.ptr());
        VL_CHECK_OGL();
    }

    // Constant color
    glColor4f(mBorderColor.r(), mBorderColor.g(), mBorderColor.b(), mBorderColor.a());

    // Constant normal
    glNormal3f( 0, 0, 1 );

    vec3 a,b,c,d;
    boundingRectTransformed( a, b, c, d, camera, mode() == Graph2D ? actor : NULL );
    fvec3 vect[] = { (fvec3)a, (fvec3)b, (fvec3)c, (fvec3)d };
    glEnableClientState( GL_VERTEX_ARRAY );
    glVertexPointer(3, GL_FLOAT, 0, vect);

    glDrawArrays(GL_LINE_LOOP, 0, 4);

    glDisableClientState( GL_VERTEX_ARRAY );

    if (mode() == Graph2D)
    {
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix(); VL_CHECK_OGL()

        glMatrixMode(GL_PROJECTION);
        glPopMatrix(); VL_CHECK_OGL()
    }
}
//-----------------------------------------------------------------------------
//! Returns the plain 2D bounding box of the text, without taking into consideration
//! the Text's matrix transform and the eventual actor's transform
AABB Graph::boundingRect() const
{
    int applied_margin = backgroundEnabled() || borderEnabled() ? margin() : 0;
    AABB bbox = rawboundingRect();
    bbox.setMaxCorner( bbox.maxCorner() + vec3(2.0f*applied_margin,2.0f*applied_margin,0) );

    // normalize coordinate orgin to the bottom/left corner
    vec3 min = bbox.minCorner() - bbox.minCorner();
    vec3 max = bbox.maxCorner() - bbox.minCorner();

    // normalize coordinate orgin to the bottom/left corner

    // alignment

    if (alignment() & AlignHCenter)
    {
        VL_CHECK( !(alignment() & AlignRight) )
        VL_CHECK( !(alignment() & AlignLeft) )
        min.x() -= int(bbox.width() / 2.0);
        max.x() -= int(bbox.width() / 2.0);
    }

    if (alignment() & AlignRight)
    {
        VL_CHECK( !(alignment() & AlignHCenter) )
        VL_CHECK( !(alignment() & AlignLeft) )
        min.x() -= (int)bbox.width();
        max.x() -= (int)bbox.width();
    }

    if (alignment() & AlignTop)
    {
        VL_CHECK( !(alignment() & AlignBottom) )
        VL_CHECK( !(alignment() & AlignVCenter) )
        min.y() -= (int)bbox.height();
        max.y() -= (int)bbox.height();
    }

    if (alignment() & AlignVCenter)
    {
        VL_CHECK( !(alignment() & AlignTop) )
        VL_CHECK( !(alignment() & AlignBottom) )
        min.y() -= int(bbox.height() / 2.0);
        max.y() -= int(bbox.height() / 2.0);
    }

    // no matrix transform applied
    // ...

    // no actor's transform applied
    // ...

    AABB aabb;
    aabb.setMinCorner(min);
    aabb.setMaxCorner(max);
    return aabb;
}
//-----------------------------------------------------------------------------
//! Returns the fully transformed bounding box.
//! \p actor is needed only if you are using the actor's transform with the Text2D
//! to make the Text2D text follow the actor on the screen or you are using the Text3D to make the text
//! follow the actor's transform in 3D.
//!
//! The layout of the a, b, c and d points is the following:\n
//! \n
//! d---------c\n
//! |         |\n
//! |         |\n
//! a---------b\n
//! \n
//! Of course the above layout can be scaled, flipped, rotated and so on according to the given Text's matrix.
AABB Graph::boundingRectTransformed(const Camera* camera, const Actor* actor) const
{
    vec3 a, b, c, d;
    return boundingRectTransformed(a, b, c, d, camera, actor);
}
//-----------------------------------------------------------------------------
AABB Graph::boundingRectTransformed(vec3& a, vec3& b, vec3& c, vec3& d, const Camera* camera, const Actor* actor) const
{
    AABB bbox = boundingRect();

    a = bbox.minCorner();
    b.x() = (float)bbox.maxCorner().x();
    b.y() = (float)bbox.minCorner().y();
    c = bbox.maxCorner();
    d.x() = (float)bbox.minCorner().x();
    d.y() = (float)bbox.maxCorner().y();
    // set z to 0
    a.z() = b.z() = c.z() = d.z() = 0;

    // viewport alignment
    fmat4 m = mMatrix;

    int w = camera->viewport()->width();
    int h = camera->viewport()->height();

    if (w < 1) w = 1;
    if (h < 1) h = 1;

    if ( !(actor && actor->transform()) && mode() == Graph2D )
    {
        if (viewportAlignment() & AlignHCenter)
        {
            VL_CHECK( !(viewportAlignment() & AlignRight) )
            VL_CHECK( !(viewportAlignment() & AlignLeft) )
            // vect[i].x() += int((viewport[2]-1.0f) / 2.0f);
            m.translate( (float)int((w-1.0f) / 2.0f), 0, 0);
        }

        if (viewportAlignment() & AlignRight)
        {
            VL_CHECK( !(viewportAlignment() & AlignHCenter) )
            VL_CHECK( !(viewportAlignment() & AlignLeft) )
            // vect[i].x() += int(viewport[2]-1.0f);
            m.translate( (float)int(w-1.0f), 0, 0);
        }

        if (viewportAlignment() & AlignTop)
        {
            VL_CHECK( !(viewportAlignment() & AlignBottom) )
            VL_CHECK( !(viewportAlignment() & AlignVCenter) )
            // vect[i].y() += int(viewport[3]-1.0f);
            m.translate( 0, (float)int(h-1.0f), 0);
        }

        if (viewportAlignment() & AlignVCenter)
        {
            VL_CHECK( !(viewportAlignment() & AlignTop) )
            VL_CHECK( !(viewportAlignment() & AlignBottom) )
            // vect[i].y() += int((viewport[3]-1.0f) / 2.0f);
            m.translate( 0, (float)int((h-1.0f) / 2.0f), 0);
        }
    }

    // ??? mix fixme: remove all these castings!
    // apply matrix transform
    a = (mat4)m * a;
    b = (mat4)m * b;
    c = (mat4)m * c;
    d = (mat4)m * d;

    // apply actor's transform
    if ( actor && actor->transform() )
    {
        if ( mode() == Graph3D )
        {
            a = actor->transform()->worldMatrix() * a;
            b = actor->transform()->worldMatrix() * b;
            c = actor->transform()->worldMatrix() * c;
            d = actor->transform()->worldMatrix() * d;
        }
        else
        if ( mode() == Graph2D )
        {
            // transform v
            vec4 v(0,0,0,1);
            v = actor->transform()->worldMatrix() * v;

            // project to screen
            camera->project(v,v);

            // from screen space to viewport space
            int viewport[] = { camera->viewport()->x(), camera->viewport()->y(), camera->viewport()->width(), camera->viewport()->height() };
            v.x() -= viewport[0];
            v.y() -= viewport[1];

            v.x() = (float)int(v.x());
            v.y() = (float)int(v.y());

            a += v.xyz();
            b += v.xyz();
            c += v.xyz();
            d += v.xyz();

            // clever trick part #2
            a.z() =
            b.z() =
            c.z() =
            d.z() = (v.z() - 0.5f) / 0.5f;
        }
    }

    bbox.setNull();
    bbox.addPoint(a);
    bbox.addPoint(b);
    bbox.addPoint(c);
    bbox.addPoint(d);
    return bbox;
}
//-----------------------------------------------------------------------------
void Graph::translate(float x, float y, float z)
{
    mMatrix.translate(x,y,z);
}
//-----------------------------------------------------------------------------
void Graph::rotate(float degrees, float x, float y, float z)
{
    mMatrix.rotate(degrees,x,y,z);
}
//-----------------------------------------------------------------------------
void Graph::resetMatrix()
{
    mMatrix.setIdentity();
}
//-----------------------------------------------------------------------------
