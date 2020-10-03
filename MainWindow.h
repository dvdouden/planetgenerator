#pragma once

#include <memory>

#include <vlGraphics/Applet.hpp>
#include <vlGraphics/Array.hpp>
#include <vlGraphics/Geometry.hpp>
#include <vlGraphics/RayIntersector.hpp>
#include <vlGraphics/Text.hpp>

#include "util/parameter.h"
#include "Planet.h"
#include "Graph.h"
#include "geom/BorderLineGeometry.h"
#include "geom/CellLineGeometry.h"
#include "geom/CellVectorGeometry.h"
#include "geom/PlanetGeometry.h"
#include "geom/RingGeometry.h"
#include "geom/PointsGeometry.h"
#include "geom/TriangleGeometry.h"
#include "geom/CentroidsGeometry.h"
#include "geom/WorldAxis.h"
#include "geom/GradientRingGeometry.h"

class MainWindow : public vl::Applet {

public:

// called once after the OpenGL window has been opened
    void initEvent() override;

    void destroyEvent() override;

    void keyPressEvent( unsigned short, vl::EKey ) override;

    void keyReleaseEvent( unsigned short, vl::EKey ) override;

    void keyPressedEvent();

    void mouseMoveEvent( int x, int y) override;

    void resizeEvent( int w, int h ) override;

    void updateScene() override;

protected:

    void generate();

    void updateGeometry();

    void updateText();

    void updateGraphs();

    vl::ref<vl::Transform> m_planetTransform;
    vl::ref<vl::Transform> m_sunRotationTransform;
    vl::ref<vl::Transform> m_sunTransform;

    vl::ref<vl::SceneManagerActorTree> m_sceneManager;

    vl::ref<vl::Effect> m_effect;

    vl::ref<vl::Text> m_text;
    vl::ref<vl::Graph> m_dailyGraph;
    vl::ref<vl::Graph> m_annualGraph;

    vl::RayIntersector m_intersector;

    std::set<vl::EKey> m_pressedKeys;

    // generator parameters
    numericParameter<int> m_pointCount = 65536;
    numericParameter<float> m_jitter = 13.9f;
    numericParameter<int> m_plateCount = 114;
    numericParameter<int> m_moisture = 0;
    numericParameter<int> m_ocean = 70;
    parameter<bool> m_useCentroids = true;
    parameter<bool> m_normalizeCentroids = true;
    numericParameter<float> m_collisionThreshold = 1.70f;

    numericParameter<float> m_timeOfDay = 0;
    numericParameter<float> m_timeOfYear = 0.0f;
    numericParameter<float> m_axialTilt = 23.5f;

    numericParameter<int> m_renderMode = 3;
    numericParameter<int> m_wireFrameRenderMode = 2;
    numericParameter<int> m_pointsRenderMode = 0;
    numericParameter<int> m_ringsRenderMode = 0;
    numericParameter<int> m_cellsRenderMode = 0;
    numericParameter<int> m_stereoFactor = 0;
    parameter<bool> m_renderCentroids = false;
    parameter<bool> m_renderPlateVectors = true;

    parameter<bool> m_pickingActive = true;
    numericParameter<vl::ivec2> m_mousePosition = vl::ivec2();
    numericParameter<int> m_highlight = -1;

    std::vector<param_ptr> m_parameters;

    double m_lastTime = 0;
    float m_deltaTimeOfDay = 0;
    float m_deltaTimeOfYear = 0;
    parameter<bool> m_timeOfDayPaused = true;
    parameter<bool> m_timeOfYearPaused = true;

    bool m_geometryInvalid = true;
    bool m_redraw = true;
    bool m_sunPositionDirty = true;


    Planet m_planet;
    vl::fvec3 m_sunRay;

    int m_triCount = 0;

    Profiler::resultset m_renderTimes;
    Profiler::resultset m_pickingTimes;
    Profiler::resultset m_lightingTimes;

    vl::ref<GradientRingGeometry> sunCircle;
    vl::ref<BorderLineGeometry> m_borderLines;
    vl::ref<CellLineGeometry> m_cellLines;
    vl::ref<CellVectorGeometry> m_cellVectors;
    vl::ref<PointsGeometry> m_pointsGeometry;
    vl::ref<TriangleGeometry> m_triangleGeometry;
    vl::ref<CentroidsGeometry> m_centroidsGeometry;
    vl::ref<PlanetGeometry> m_planetGeometry;
    vl::ref<WorldAxis> m_sunAxis;
    vl::ref<WorldAxis> m_planetAxis;
    std::vector<vl::ref<RingGeometry>> m_planetRingGeometries;
};

