#pragma once

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


#include <vlGraphics/Font.hpp>
#include <vlGraphics/Renderable.hpp>
#include <vlCore/vlnamespace.hpp>
#include <vlCore/String.hpp>
#include <vlCore/Rect.hpp>
#include <map>

namespace vl
{

    typedef enum
    {
        Graph2D = 1,
        Graph3D = 2
    } EGraphMode;

    /**
     * A Renderable that renders a graph.
     * \sa
     * - Actor
     * - VectorGraphics
    */
    class VLGRAPHICS_EXPORT Graph: public Renderable
    {
    VL_INSTRUMENT_CLASS(vl::Graph, Renderable)

    public:
        Graph(): mColor(1,1,1,1), mBorderColor(0,0,0,1), mBackgroundColor(1,1,1,1), mOutlineColor(0,0,0,1), mShadowColor(0,0,0,0.5f), mShadowVector(2,-2), mMarkerColor(1,1,1,1),
                mMarkerPosition(0),mInterlineSpacing(5), mAlignment(AlignBottom|AlignLeft), mViewportAlignment(AlignBottom|AlignLeft), mMargin(5), mMode(Graph2D),
                mBorderEnabled(false), mBackgroundEnabled(false), mOutlineEnabled(false), mShadowEnabled(false), mMarkerEnabled(false)
        {
            VL_DEBUG_SET_OBJECT_NAME()
        }

        void setData( const std::vector<vl::fvec2>& data ) { mData = data; }
        const std::vector<vl::fvec2>& data() const { return mData; }

        void setXMinMax( float xMin, float xMax ) {
            mMin.x() = xMin;
            mMax.x() = xMax;
        }

        void setYMinMax( float yMin, float yMax ) {
            mMin.y() = yMin;
            mMax.y() = yMax;
        }

        const fvec4& color() const { return mColor; }
        void setColor(const fvec4& color) { mColor = color; }

        const fvec4& borderColor() const { return mBorderColor; }
        void setBorderColor(const fvec4& border_color) { mBorderColor = border_color; }

        const fvec4& outlineColor() const { return mOutlineColor; }
        void setOutlineColor(const fvec4& outline_color) { mOutlineColor = outline_color; }

        const fvec4& backgroundColor() const { return mBackgroundColor; }
        void setBackgroundColor(const fvec4& background_color) { mBackgroundColor = background_color; }

        const fvec4& shadowColor() const { return mShadowColor; }
        void setShadowColor(const fvec4& shadow_color) { mShadowColor = shadow_color; }

        const fvec2& shadowVector() const { return mShadowVector; }
        void setShadowVector(const fvec2& shadow_vector) { mShadowVector = shadow_vector; }

        const fvec4& markerColor() const { return mMarkerColor; }
        void setMarkerColor(const fvec4& marker_color) { mMarkerColor = marker_color; }

        const float markerPosition() const { return mMarkerPosition; }
        void setMarkerPosition( float marker_position ) { mMarkerPosition = marker_position; }

        int margin() const { return mMargin; }
        void setMargin(int margin) { mMargin = margin; }

        const fmat4& matrix() const { return mMatrix; }
        void setMatrix(const fmat4& matrix) { mMatrix = matrix; }

        unsigned int  alignment() const { return mAlignment; }
        void setAlignment(unsigned int  align) { mAlignment = align; }

        unsigned int  viewportAlignment() const { return mViewportAlignment; }
        void setViewportAlignment(unsigned int  align) { mViewportAlignment = align; }

        float interlineSpacing() const { return mInterlineSpacing; }
        void setInterlineSpacing(float spacing) { mInterlineSpacing = spacing; }

        EGraphMode mode() const { return mMode; }
        void setMode(EGraphMode mode) { mMode = mode; }

        bool borderEnabled() const { return mBorderEnabled; }
        void setBorderEnabled(bool border) { mBorderEnabled = border; }

        bool backgroundEnabled() const { return mBackgroundEnabled; }
        void setBackgroundEnabled(bool background) { mBackgroundEnabled = background; }

        bool outlineEnabled() const { return mOutlineEnabled; }
        void setOutlineEnabled(bool outline) { mOutlineEnabled = outline; }

        bool shadowEnabled() const { return mShadowEnabled; }
        void setShadowEnabled(bool shadow) { mShadowEnabled = shadow; }

        bool markerEnabled() const { return mMarkerEnabled; }
        void setMarkerEnabled(bool marker) { mMarkerEnabled = marker; }

        void render_Implementation(const Actor* actor, const Shader* shader, const Camera* camera, OpenGLContext* gl_context) const override;
        void computeBounds_Implementation() override {
            setBoundingBox(AABB());
            setBoundingSphere(Sphere());
        }
        AABB boundingRect() const;
        AABB boundingRectTransformed(vec3& a, vec3& b, vec3& c, vec3& d, const Camera* camera, const Actor* actor=NULL) const;
        AABB boundingRectTransformed(const Camera* camera, const Actor* actor=NULL) const;

        void translate(float x, float y, float z);
        void rotate(float degrees, float x, float y, float z);
        void resetMatrix();

        // Renderable interface implementation.

        void updateDirtyBufferObject(EBufferObjectUpdateMode) override {}

        void deleteBufferObject() override {}

        void setSize( unsigned int width, unsigned int height ) {
            mWidth = width;
            mHeight = height;
        }

    protected:
        void renderGraph(const Actor*, const Camera* camera, const fvec4& color, const fvec2& offset) const;
        void renderBackground(const Actor* actor, const Camera* camera) const;
        void renderBorder(const Actor* actor, const Camera* camera) const;
        AABB rawboundingRect() const;

    protected:
        fvec4 mColor;
        fvec4 mBorderColor;
        fvec4 mBackgroundColor;
        fvec4 mOutlineColor;
        fvec4 mShadowColor;
        fvec2 mShadowVector;
        fvec4 mMarkerColor;
        float mMarkerPosition;
        fmat4 mMatrix;
        float mInterlineSpacing;
        unsigned int mAlignment;
        unsigned int mViewportAlignment;
        int mMargin;
        EGraphMode mMode;
        bool mBorderEnabled;
        bool mBackgroundEnabled;
        bool mOutlineEnabled;
        bool mShadowEnabled;
        bool mMarkerEnabled;
        std::vector<vl::fvec2> mData;
        vl::fvec2 mMin;
        vl::fvec2 mMax;
        unsigned int mWidth;
        unsigned int mHeight;
    };
}
