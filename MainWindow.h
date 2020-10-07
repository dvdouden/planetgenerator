#pragma once

#include <memory>

#include <vlGraphics/Applet.hpp>
#include <vlGraphics/Array.hpp>
#include <vlGraphics/Geometry.hpp>
#include <vlGraphics/Text.hpp>

#include "util/parameter.h"
#include "Planet.h"
#include "PlanetIntersector.h"
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
#include "geom/PlateOriginGeometry.h"

class MainWindow : public vl::Applet {

public:
    MainWindow() :
    vl::Applet(),
    m_planetIntersector( m_planet ) {}

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

    std::set<vl::EKey> m_pressedKeys;

    numericParameter<float> m_timeOfDay = { 0, "time of day" };
    numericParameter<float> m_timeOfYear = { 0.0f, "time of year" };

    numericParameter<int> m_wireFrameRenderMode = {2, "wireframe render mode" };
    numericParameter<int> m_pointsRenderMode = { 0, "points render mode" };
    numericParameter<int> m_ringsRenderMode = { 0, "rings render mode" };
    numericParameter<int> m_cellsRenderMode = { 0, "cells render mode" };
    numericParameter<int> m_stereoFactor = { 0, "stereographic projection factor" };
    boolParameter m_renderCentroids = { false, "render centroids" };
    boolParameter m_renderPlateVectors = { true, "render plate vectors" };
    boolParameter m_renderPlateOrigins = { true, "render plate origins" };
    boolParameter m_renderPlanetAxis = { true, "render planet axis" };

    boolParameter m_pickingActive = { true, "picking active" };
    numericParameter<vl::ivec2> m_mousePosition = { vl::ivec2(), "mouse position" };
    numericParameter<int> m_highlight = { -1, "highlight" };

    std::vector<std::vector<param_ptr>> m_parameters;
    std::vector<std::map<vl::EKey,param_ptr>> m_keyBindings;

    double m_lastTime = 0;
    float m_deltaTimeOfDay = 0;
    float m_deltaTimeOfYear = 0;
    boolParameter m_timeOfDayPaused = { true, "time of day paused" };
    boolParameter m_timeOfYearPaused = { true, "time of year paused" };

    bool m_geometryInvalid = true;
    bool m_sunPositionDirty = true;


    Planet m_planet;
    PlanetIntersector m_planetIntersector;
    vl::fvec3 m_sunRay;

    int m_triCount = 0;

    Profiler::resultset m_renderTimes;
    Profiler::resultset m_pickingTimes;
    Profiler::resultset m_lightingTimes;

    vl::ref<GradientRingGeometry> m_sunCircle;
    vl::ref<BorderLineGeometry> m_borderLines;
    vl::ref<CellLineGeometry> m_cellLines;
    vl::ref<CellVectorGeometry> m_cellVectors;
    vl::ref<PointsGeometry> m_pointsGeometry;
    vl::ref<TriangleGeometry> m_triangleGeometry;
    vl::ref<CentroidsGeometry> m_centroidsGeometry;
    vl::ref<PlanetGeometry> m_planetGeometry;
    vl::ref<PlateOriginGeometry> m_plateOriginGeometry;
    vl::ref<WorldAxis> m_sunAxis;
    vl::ref<WorldAxis> m_planetAxis;
    std::vector<vl::ref<RingGeometry>> m_planetRingGeometries;
};

