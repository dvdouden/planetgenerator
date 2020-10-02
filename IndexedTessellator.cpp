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

#include "IndexedTessellator.h"

using namespace vl;

//-----------------------------------------------------------------------------
IndexedTessellator::IndexedTessellator()
{
    VL_DEBUG_SET_OBJECT_NAME()
    mTessNormal = fvec3(0,1,0);
    mBoundaryOnly = false;
    mTolerance = 0.0;
    mWindingRule = TW_TESS_WINDING_ODD;
    mTessellateIntoSinglePolygon = true;
}
//-----------------------------------------------------------------------------
IndexedTessellator::~IndexedTessellator()
{
    freeCombinedVertices();
}
//-----------------------------------------------------------------------------
bool IndexedTessellator::tessellate(bool append_tessellated_tris)
{
    if (!append_tessellated_tris)
        mTessellatedTris.clear();
    mFans.clear();
    mTriStrips.clear();
    mLineLoops.clear();
    mPrimitiveType = 0;
    freeCombinedVertices();
    if (mContours.empty() || mContourVerts.empty())
    {
        vl::Log::error("IndexedTessellator::tessellate(): no contours specified.\n");
        return false;
    }

    GLUtesselator* tobj = gluNewTess();
    // callbacks
    gluTessCallback(tobj, GLU_TESS_BEGIN_DATA,   (callback_type)tessBeginData);
    gluTessCallback(tobj, GLU_TESS_VERTEX_DATA,  (callback_type)tessVertexData);
    gluTessCallback(tobj, GLU_TESS_COMBINE_DATA, (callback_type)tessCombineData);
    gluTessCallback(tobj, GLU_TESS_END,     (callback_type)tessEnd);
    gluTessCallback(tobj, GLU_TESS_ERROR,        (callback_type)tessError);
    // normal
    gluTessNormal( tobj, tessNormal().x(), tessNormal().y(), tessNormal().z() );
    // properties
    gluTessProperty(tobj, GLU_TESS_BOUNDARY_ONLY, boundaryOnly() ? GL_TRUE : GL_FALSE);
    gluTessProperty(tobj, GLU_TESS_TOLERANCE, tolerance());
    gluTessProperty(tobj, GLU_TESS_WINDING_RULE, windingRule());
    // tessellation
    if (tessellateIntoSinglePolygon())
    {
        gluTessBeginPolygon(tobj, this);
        for(unsigned cont=0, idx=0; cont<mContours.size(); ++cont)
        {
            gluTessBeginContour(tobj);
            for(int i=0; i<mContours[cont]; ++i, ++idx) {
                //printf( "Pushing idx %d (=%d) %f %f %f\n", idx, mIndices[idx], mContourVerts[idx].x(), mContourVerts[idx].y(), mContourVerts[idx].z() );
                gluTessVertex(tobj, mContourVerts[idx].ptr(), (void*)mIndices[idx]);
            }
            gluTessEndContour(tobj);
        }
        gluTessEndPolygon(tobj);
    }
    else
    {
        for(unsigned cont=0, idx=0; cont<mContours.size(); ++cont)
        {
            gluTessBeginPolygon(tobj, this);
            gluTessBeginContour(tobj);
            for(int i=0; i<mContours[cont]; ++i, ++idx) {
                //printf( "Pushing idx %d (=%d) %f %f %f\n", idx, mIndices[idx], mContourVerts[idx].x(), mContourVerts[idx].y(), mContourVerts[idx].z() );
                gluTessVertex( tobj, mContourVerts[idx].ptr(), (void*)mIndices[idx] );
            }
            gluTessEndContour(tobj);
            gluTessEndPolygon(tobj);
        }
    }
    gluDeleteTess(tobj);

    // triangulate fans
    for(unsigned fan=0; fan<mFans.size();    ++fan)
        for(unsigned iv =1; iv<mFans[fan].size()-1; ++iv)
        {
            //printf( "push fan %d\n", mFans[fan][0]);
            mTessellatedTris.push_back(mFans[fan][0]);
            //printf( "push fan %d\n", mFans[fan][iv]);
            mTessellatedTris.push_back(mFans[fan][iv]);
            //printf( "push fan %d\n", mFans[fan][iv + 1]);
            mTessellatedTris.push_back(mFans[fan][iv+1]);
        }

    // triangulate strips
    for(unsigned strip=0; strip<mTriStrips.size(); ++strip)
        for(unsigned iv=0; iv<mTriStrips[strip].size()-2; ++iv)
        {
            if (iv % 2)
            {
               // printf( "push strip %d\n", mTriStrips[strip][iv + 0]);
                mTessellatedTris.push_back(mTriStrips[strip][iv+0]);
                //printf( "push strip %d\n", mTriStrips[strip][iv + 2]);
                mTessellatedTris.push_back(mTriStrips[strip][iv+2]);
                //printf( "push strip %d\n", mTriStrips[strip][iv + 1]);
                mTessellatedTris.push_back(mTriStrips[strip][iv+1]);
            }
            else
            {
                //printf( "push strip %d\n", mTriStrips[strip][iv + 0]);
                mTessellatedTris.push_back(mTriStrips[strip][iv+0]);
                //printf( "push strip %d\n", mTriStrips[strip][iv + 1]);
                mTessellatedTris.push_back(mTriStrips[strip][iv+1]);
                //printf( "push strip %d\n", mTriStrips[strip][iv + 2]);
                mTessellatedTris.push_back(mTriStrips[strip][iv+2]);
            }
        }

    mContours.clear();
    mContourVerts.clear();

#if 0
    if (tessellatedTris().empty())
    {
      vl::Log::warning("Tessellator::tessellate(): no triangles generated.\n");
      return false;
    }
#endif

    return true;
}
//-----------------------------------------------------------------------------
void IndexedTessellator::freeCombinedVertices()
{
    mCombinedVertices.clear();
}

//-----------------------------------------------------------------------------
// Tessellation callbacks
//-----------------------------------------------------------------------------
void CALLBACK IndexedTessellator::tessBeginData( GLenum type, IndexedTessellator* tessellator )
{
    tessellator->mPrimitiveType = type;
    if(type == GL_TRIANGLES)
    {
        //printf( "begin triangles\n" );
        // do nothing
    }
    else
    if(type == GL_TRIANGLE_FAN) {
       // printf( "begin fan\n" );
        tessellator->mFans.resize( tessellator->mFans.size() + 1 );
    }
    else
    if(type == GL_TRIANGLE_STRIP) {
        //printf( "begin strip\n" );
        tessellator->mTriStrips.resize( tessellator->mTriStrips.size() + 1 );
    }
    else
    if(type == GL_LINE_LOOP) {
        //printf( "begin loop\n" );
        tessellator->mLineLoops.resize( tessellator->mLineLoops.size() + 1 );
    }
    else
    {
        Log::error("Tessellator::beginData() unknown primitive.\n");
    }
}
//-----------------------------------------------------------------------------
void CALLBACK IndexedTessellator::tessVertexData( int vec, IndexedTessellator* tessellator )
{
    if(tessellator->mPrimitiveType == GL_TRIANGLES) {
        //printf( "data tri %d\n", vec );
        tessellator->mTessellatedTris.push_back( vec ); }
    else
    if(tessellator->mPrimitiveType == GL_TRIANGLE_FAN) {
        //printf( "data fan %d\n", vec );
        tessellator->mFans.back().push_back( vec );
    }
    else
    if(tessellator->mPrimitiveType == GL_TRIANGLE_STRIP) {
        //printf( "data strip %d\n", vec );
        tessellator->mTriStrips.back().push_back( vec );
    }
    else
    if(tessellator->mPrimitiveType == GL_LINE_LOOP) {
        //printf( "data loop %d\n", vec );
        tessellator->mLineLoops.back().push_back( vec );
    }
    else
    {
        Log::error("Tessellator::vertexData() unknown primitive.\n");
    }
}
//-----------------------------------------------------------------------------
void CALLBACK IndexedTessellator::tessCombineData( GLdouble coords[3], dvec3*[4], GLfloat[4], void **dataOut, IndexedTessellator* tessellator )
{
    //printf( "Combine data?\n" );
    tessellator->mCombinedVertices.emplace_back( coords[0], coords[1], coords[2] );
    // use negative index to indicate a new vertex
    *dataOut = (void*)-tessellator->mCombinedVertices.size();
    //printf( "out %d = %f %f %f\n", -tessellator->mCombinedVertices.size(), coords[0], coords[1], coords[2]);
}
//-----------------------------------------------------------------------------
void CALLBACK IndexedTessellator::tessEnd(void)
{
    //printf( "end\n" );
}
//-----------------------------------------------------------------------------
void CALLBACK IndexedTessellator::tessError( GLenum errno )
{
    const GLubyte* estring = gluErrorString(errno);
    Log::error( Say("Tessellator error: %s.\n") << estring );
}
//-----------------------------------------------------------------------------
