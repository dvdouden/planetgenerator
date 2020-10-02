#include <vlCore/Time.hpp>
#include <vlGraphics/FontManager.hpp>
#include <vlGraphics/GeometryPrimitives.hpp>

#include <thread>

#include "MainWindow.h"
#include "util/fmath.h"

void MainWindow::initEvent() {
    /* black background */
    rendering()->as<vl::Rendering>()->camera()->viewport()->setClearColor( vl::black );

    /* define the camera position and orientation */
    vl::real distance = 2.0;
    vl::vec3 eye = vl::vec3( distance, 0, distance ); // camera position
    vl::vec3 center = vl::vec3( 0, 0, 0 );   // point the camera is looking at
    vl::vec3 up = vl::vec3( 0, 1, 0 );   // up direction
    vl::mat4 view_mat = vl::mat4::getLookAt( eye, center, up );

    rendering()->as<vl::Rendering>()->camera()->setViewMatrix( view_mat );

    vl::ref<vl::Effect> text_fx = new vl::Effect;
    text_fx->shader()->enable( vl::EN_BLEND );
    m_text = new vl::Text;
    m_text->setFont( vl::defFontManager()->acquireFont( "/fonts/typed.ttf", 10 ) );
    m_text->setAlignment( vl::AlignLeft | vl::AlignTop );
    m_text->setViewportAlignment( vl::AlignLeft | vl::AlignTop );
    m_text->translate( 0, -10, 0 );
    sceneManager()->tree()->addActor( m_text.get(), text_fx.get() )->setObjectName( "HUD" );
    updateText();

    vl::ref<vl::Effect> graph_fx = new vl::Effect;
    graph_fx->shader()->enable( vl::EN_BLEND );
    m_dailyGraph = new vl::Graph;
    m_dailyGraph->setColor( vl::fvec4( 1.0, 0.5, 0.5, 1.0 ) );
    m_dailyGraph->setMarkerColor( vl::fvec4( 1.0, 0, 0, 1.0 ) );
    m_dailyGraph->setAlignment( vl::AlignRight | vl::AlignBottom );
    m_dailyGraph->setViewportAlignment( vl::AlignRight | vl::AlignBottom );
    m_dailyGraph->setSize( 1280/2, 200 );
    m_dailyGraph->setXMinMax( 0, 1);
    m_dailyGraph->setYMinMax( 0, 1);
    m_dailyGraph->setMarkerEnabled( true );
    sceneManager()->tree()->addActor( m_dailyGraph.get(), graph_fx.get() )->setObjectName( "Graph" );

    m_annualGraph = new vl::Graph;
    m_annualGraph->setColor( vl::fvec4( 0.5, 1.0, 0.5, 1.0 ) );
    m_annualGraph->setMarkerColor( vl::fvec4( 0, 1.0, 0, 1.0 ) );
    m_annualGraph->setAlignment( vl::AlignLeft | vl::AlignBottom );
    m_annualGraph->setViewportAlignment( vl::AlignLeft | vl::AlignBottom );
    m_annualGraph->setSize( 1280/2, 200 );
    m_annualGraph->setXMinMax( 0, 1);
    m_annualGraph->setYMinMax( 0, 0.5f);
    m_annualGraph->setMarkerEnabled( true );
    sceneManager()->tree()->addActor( m_annualGraph.get(), graph_fx.get() )->setObjectName( "Graph" );

    updateGraphs();

    // allocate the Transform
    m_planetTransform = new vl::Transform;
    // bind the Transform with the transform tree of the rendring pipeline
    rendering()->as<vl::Rendering>()->transform()->addChild( m_planetTransform.get() );

    // install our scene manager, we use the SceneManagerActorTree which is the most generic
    m_sceneManager = new vl::SceneManagerActorTree;
    rendering()->as<vl::Rendering>()->sceneManagers()->push_back( m_sceneManager.get());

    // setup the effect to be used to render the cube
    m_effect = new vl::Effect;
    // enable depth test and lighting
    m_effect->shader()->gocPointSize()->set( 5.0f);
    m_effect->shader()->enable( vl::EN_DEPTH_TEST);
    m_effect->shader()->enable( vl::EN_CULL_FACE);
    m_effect->shader()->enable( vl::EN_BLEND);
    m_effect->shader()->gocHint()->setPointSmoothHint( vl::HM_FASTEST);

    m_sunTransform = new vl::Transform;
    m_sunRotationTransform = new vl::Transform;
    m_sunRotationTransform->addChild( m_sunTransform.get() );
    rendering()->as<vl::Rendering>()->transform()->addChild( m_sunRotationTransform.get() );

    m_sunAxis = new WorldAxis();
    m_sceneManager->tree()->addActor( m_sunAxis.get(), m_effect.get(), m_sunTransform.get() )->setObjectName( "Sun");
    m_sunAxis->setEnabled( false );
    m_sunAxis->update();

    sunCircle = new GradientRingGeometry();
    m_sceneManager->tree()->addActor( sunCircle.get(), m_effect.get(), m_sunRotationTransform.get() )->setObjectName( "Sun Path");
    sunCircle->setEnabled( true );

    m_sunRay = m_sunTransform->localMatrix() * vl::fvec3( 1, 0, 0);

    m_borderLines = new BorderLineGeometry( m_planet );
    m_sceneManager->tree()->addActor( m_borderLines.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "Borderlines" );
    m_cellLines = new CellLineGeometry( m_planet );
    m_sceneManager->tree()->addActor( m_cellLines.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "Celllines" );
    m_cellVectors = new CellVectorGeometry( m_planet );
    m_sceneManager->tree()->addActor( m_cellVectors.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "Cellvectors" );
    m_pointsGeometry = new PointsGeometry( m_planet );
    m_sceneManager->tree()->addActor( m_pointsGeometry.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "Points" );
    m_triangleGeometry = new TriangleGeometry( m_planet );
    m_sceneManager->tree()->addActor( m_triangleGeometry.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "Triangles" );
    m_centroidsGeometry = new CentroidsGeometry( m_planet );
    m_sceneManager->tree()->addActor( m_centroidsGeometry.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "Centroids" );
    m_planetGeometry = new PlanetGeometry( m_planet );
    auto* planetActor = m_sceneManager->tree()->addActor( m_planetGeometry.get(), m_effect.get(), m_planetTransform.get() );
    planetActor->setObjectName( "Planet" );
    m_intersector.actors()->push_back( planetActor );
    m_planetRingGeometries.push_back( new RingGeometry );
    m_sceneManager->tree()->addActor( m_planetRingGeometries.back().get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "ArcticCircle" );
    m_planetRingGeometries.push_back( new RingGeometry );
    m_sceneManager->tree()->addActor( m_planetRingGeometries.back().get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "TropicOfCancer" );
    m_planetRingGeometries.push_back( new RingGeometry );
    m_sceneManager->tree()->addActor( m_planetRingGeometries.back().get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "Equator" );
    m_planetRingGeometries.push_back( new RingGeometry );
    m_sceneManager->tree()->addActor( m_planetRingGeometries.back().get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "TropicOfCapricorn" );
    m_planetRingGeometries.push_back( new RingGeometry );
    m_sceneManager->tree()->addActor( m_planetRingGeometries.back().get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "AntarcticCircle" );

    m_planetAxis = new WorldAxis();
    m_sceneManager->tree()->addActor( m_planetAxis.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "PlanetAxis");
    m_planetAxis->setEnabled( true );
    m_planetAxis->update();

    generate();
    updateGeometry();

    // set up key bindings for parameters
    m_parameters.emplace_back( param_ptr( new parameterBinding<int>( m_plateCount, 0, 10000, 1, vl::Key_Insert, vl::Key_Delete, true )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<float>( m_jitter, 0.0f, 360.0f, 1.0f, vl::Key_Plus, vl::Key_Minus, true )));
    m_parameters.emplace_back( param_ptr( new boolParameterBinding( m_useCentroids, vl::Key_BackSlash, true )));
    m_parameters.emplace_back( param_ptr( new boolParameterBinding( m_normalizeCentroids, vl::Key_N, true )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<int>( m_wireFrameRenderMode, 0, 9, 1, vl::Key_W, vl::Key_Q, false )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<int>( m_pointsRenderMode, 0, 9, 1, vl::Key_P, vl::Key_O, false )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<int>( m_cellsRenderMode, 0, 9, 1, vl::Key_R, vl::Key_E, false )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<int>( m_ringsRenderMode, 0, 9, 1, vl::Key_X, vl::Key_Z, false )));
    m_parameters.emplace_back( param_ptr( new boolParameterBinding( m_renderCentroids, vl::Key_C, false )));
    m_parameters.emplace_back( param_ptr( new boolParameterBinding( m_renderPlateVectors, vl::Key_V, false )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<int>( m_stereoFactor, 0, 100, 1, vl::Key_RightBracket, vl::Key_LeftBracket, false )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<float>( m_collisionThreshold, 0.0f, 4.0f, 0.05f, vl::Key_Period, vl::Key_Comma, true )));
    m_parameters.emplace_back( param_ptr( new boolParameterBinding( m_pickingActive, vl::Key_Space, false )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<int>( m_moisture, 0, 100, 1, vl::Key_Quote, vl::Key_Semicolon, false )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<float>( m_axialTilt, 0.0f, 180.0f, 0.5f, vl::Key_Home, vl::Key_End, false )));
    m_parameters.emplace_back( param_ptr( new parameterBinding<float>( m_timeOfYear, 0.0f, 1.0f, 0.0625f, vl::Key_L, vl::Key_K, false )));
    m_parameters.emplace_back( param_ptr( new boolParameterBinding( m_timeOfDayPaused, vl::Key_T, false )));
    m_parameters.emplace_back( param_ptr( new boolParameterBinding( m_timeOfYearPaused, vl::Key_Y, false )));
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 1, vl::Key_1, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 2, vl::Key_2, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 3, vl::Key_3, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 4, vl::Key_4, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 5, vl::Key_5, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 6, vl::Key_6, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 7, vl::Key_7, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 8, vl::Key_8, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 9, vl::Key_9, false ) ) );
    m_parameters.emplace_back( param_ptr( new constantParameterBinding<int>( m_renderMode, 0, vl::Key_0, false ) ) );
}


void MainWindow::destroyEvent() {

}

void MainWindow::resizeEvent( int w, int h ) {
    Applet::resizeEvent(w, h);
    m_dailyGraph->setSize( w / 2, h / 6 );
    m_annualGraph->setSize( w / 2, h / 6 );
}


void MainWindow::keyPressEvent( unsigned short ch, vl::EKey key ) {
    m_pressedKeys.insert( key );
    bool regenerate = false;
    for ( auto& parameter : m_parameters ) {
        if ( key == parameter->incKey ) {
            parameter->inc();
            regenerate = regenerate || parameter->generate;
        } else if ( key == parameter->decKey ) {
            parameter->dec();
            regenerate = regenerate || parameter->generate;
        }
    }
    switch ( key ) {

        case vl::Key_PageUp:
            m_pointCount = m_pointCount() * 2;
            generate();
            break;

        case vl::Key_PageDown:
            m_pointCount = m_pointCount() / 2;
            generate();
            break;

        default:
            Applet::keyPressEvent( ch, key );
            break;
    }

    if ( regenerate ) {
        generate();
    }
    m_redraw = true;
}

void MainWindow::keyPressedEvent() {
    for ( vl::EKey key : m_pressedKeys ) {
        switch ( key ) {
            case vl::Key_Left:
                m_pointCount -= 1;
                generate();
                break;

            case vl::Key_Right:
                m_pointCount += 1;
                generate();
                break;

            case vl::Key_Up:
                m_pointCount += 10;
                generate();
                break;

            case vl::Key_Down:
                m_pointCount -= 10;
                generate();
                break;

            default:
                break;
        }
    }
}

void MainWindow::keyReleaseEvent( unsigned short ch, vl::EKey key ) {
    m_pressedKeys.erase( key );
    switch ( key ) {
        // we want to use these keys for other purposes (and it'll break stuff if we let the default behavior kick in)
        case vl::Key_F: // camera controls
        case vl::Key_T: // don't touch my camera!
        case vl::Key_C: // don't touch my camera!
            break;

        default:
            Applet::keyReleaseEvent( ch, key );
            break;
    }
    m_redraw = true;
}

void MainWindow::mouseMoveEvent(int x, int y) {
    m_mousePosition = vl::ivec2( x, y );

    Applet::mouseMoveEvent( x, y );
}

// called every frame
void MainWindow::updateScene() {
    keyPressedEvent();

    if ( m_stereoFactor.dirty ) {
        m_pointsGeometry->setStereo( m_stereoFactor() / 100.0f );
        m_triangleGeometry->setStereo( m_stereoFactor() / 100.0f );
        m_centroidsGeometry->setStereo( m_stereoFactor() / 100.0f );
        m_stereoFactor.clear();
    }

    if ( m_pickingActive() && m_renderMode() == 3 && m_mousePosition.dirty && !m_geometryInvalid ) {
        m_mousePosition.clear();
        Profiler profiler( "Picking" );
        int x = m_mousePosition().x();
        int y = m_mousePosition().y();
        vl::Camera* camera = rendering()->as<vl::Rendering>()->camera();
        y = openglContext()->height() - y;
        vl::Ray ray = camera->computeRay(x,y);
        m_intersector.setFrustum( camera->computeRayFrustum( x,y ) );
        m_intersector.setRay(ray);
        m_intersector.intersect();
        // run intersection test
        profiler("Check intersections" );
        if (!m_intersector.intersections().empty()) {
            // this is a bit nasty, but since we only have an index, we need to loop through all cells until we know which cell it belongs to...
            int idx = m_intersector.intersections()[0]->as<vl::RayIntersectionGeometry>()->triangleIndex();

            int c = 0;
            for ( const auto& cell : m_planet.cells ) {
                if ( idx >= c && idx < c + cell.edges.size() ) {
                    if ( m_highlight() != cell.point ) {
                        m_redraw = true;
                        m_highlight = cell.point;
                    }
                    break;
                }
                c += cell.edges.size();
            }
            profiler("find cell");
        }
        m_pickingTimes = profiler.results();
    }
    double currentTime = vl::Time::currentTime();
    if ( m_lastTime == 0 ) {
        m_lastTime = currentTime;
    }
    double deltaTime = currentTime - m_lastTime;
    m_lastTime = currentTime;

    double tmp;
    float secondsPerYear = 120;
    float secondsPerDay = secondsPerYear / 365.25f;
    if ( !m_timeOfDayPaused() ) {
        m_deltaTimeOfDay = (float)deltaTime / secondsPerDay;
        m_timeOfDay += m_deltaTimeOfDay;
        m_timeOfDay = (float)modf( m_timeOfDay(), &tmp );
        m_sunPositionDirty = true;
    }
    if ( !m_timeOfYearPaused() ) {
        m_deltaTimeOfYear = (float)deltaTime / secondsPerYear;
        m_timeOfYear += m_deltaTimeOfYear;
        m_timeOfYear = (float)modf( m_timeOfYear(), &tmp );
        m_sunPositionDirty = true;
    }

    if ( m_axialTilt.dirty ) {
        m_sunPositionDirty = true;
        m_planet.axialTilt = m_axialTilt();
        m_planetRingGeometries[0]->setLatitude( 90 - m_planet.axialTilt );
        m_planetRingGeometries[1]->setLatitude( m_planet.axialTilt );
        // equator doesn't change (duh)
        m_planetRingGeometries[3]->setLatitude( -m_planet.axialTilt );
        m_planetRingGeometries[4]->setLatitude( -90 + m_planet.axialTilt );
    }

    if ( m_sunPositionDirty ) {
        double radTOY = (m_timeOfYear() + 0.5) * 2 *
                        vl::dPi; // add a half, we'd like the year to start with winter in the northern hemisphere
        vl::mat4 tiltMtx = vl::mat4::getRotation( m_axialTilt(), sin( radTOY ) * vl::dRAD_TO_DEG, 0,
                                                  cos( radTOY ) * vl::dRAD_TO_DEG );

        double degTOD = m_timeOfDay() * 360;
        vl::mat4 rotMtx = vl::mat4::getRotation( -degTOD, 0, 1, 0 );

        m_sunTransform->setLocalMatrix( tiltMtx );
        m_sunRotationTransform->setLocalMatrix( rotMtx );
        m_sunRotationTransform->computeWorldMatrix();
        m_sunRay = m_sunTransform->worldMatrix() * vl::fvec3( 1, 0, 0 );

        sunCircle->setLatitude( m_axialTilt() * cos( radTOY ) );
        sunCircle->update();
    }

    if ( (m_cellsRenderMode() == 6 || m_cellsRenderMode() == 8) && m_sunPositionDirty ) {
        Profiler profiler("lighting and heat");
        m_planet.calcLight( m_sunRay );
        profiler( "light" );
        m_planet.updateTemperature( m_deltaTimeOfDay );
        profiler( "temperature" );
        m_lightingTimes = profiler.results();
        m_redraw = true;
    }


    if ( m_sunPositionDirty ) {
        m_borderLines->markColorsDirty();
        m_cellLines->markColorsDirty();
        if ( m_cellsRenderMode() == 6 || m_cellsRenderMode() == 8) {
            m_planetGeometry->markColorsDirty();
        }
    }
    if ( m_cellsRenderMode.dirty ) {
        m_planetGeometry->setColorMode( m_cellsRenderMode() );
        m_planetGeometry->markColorsDirty();
    }
    m_planetGeometry->enablePicking( m_pickingActive() );
    if ( m_highlight.dirty ) {
        m_planetGeometry->setHighlight( m_highlight() );
        m_highlight.clear();
    }

    if ( m_redraw ) {
        updateGeometry();
    }
    updateText();
    updateGraphs();
    m_sunPositionDirty = false;
    m_axialTilt.clear();
}


void MainWindow::updateGeometry() {
    Profiler profiler( "Renderer" );

    if ( m_geometryInvalid ) {
        m_planet.generate( m_pointCount(), m_jitter(), m_useCentroids(), m_normalizeCentroids(), m_plateCount(), m_collisionThreshold(), m_moisture() / 100.0f );

        m_borderLines->markGeometryInvalid();
        m_cellLines->markGeometryInvalid();
        m_cellVectors->markGeometryInvalid();
        m_pointsGeometry->markGeometryInvalid();
        m_triangleGeometry->markGeometryInvalid();
        m_centroidsGeometry->markGeometryInvalid();
        m_planetGeometry->markGeometryInvalid();

        m_geometryInvalid = false;
    }


    m_pointsGeometry->setEnabled( m_renderMode() == 1 );
    m_pointsGeometry->update();
    profiler( "Points" );

    m_triangleGeometry->setEnabled( m_renderMode() == 2 );
    m_triangleGeometry->update();
    profiler( "Triangles" );

    m_centroidsGeometry->setEnabled( m_renderCentroids() );
    m_centroidsGeometry->update();
    profiler( "Centroids" );

    m_borderLines->setEnabled( m_renderMode() == 3 && m_wireFrameRenderMode() > 0 );
    m_borderLines->update();
    profiler( "Plateborders" );

    m_cellLines->setEnabled( m_renderMode() == 3 && m_wireFrameRenderMode() > 1 );
    m_cellLines->update();
    profiler( "Cellborders" );

    m_cellVectors->setEnabled( m_renderMode() == 3 && m_renderPlateVectors() );
    m_cellVectors->update();
    profiler( "Cellvectors" );

    m_planetGeometry->setEnabled( m_renderMode() == 3 );
    m_planetGeometry->update();
    profiler( "Planet" );

    m_planetRingGeometries[0]->setEnabled( m_renderMode() == 3 && m_ringsRenderMode() > 1 );
    m_planetRingGeometries[1]->setEnabled( m_renderMode() == 3 && m_ringsRenderMode() > 1 );
    m_planetRingGeometries[2]->setEnabled( m_renderMode() == 3 && m_ringsRenderMode() > 0 );
    m_planetRingGeometries[3]->setEnabled( m_renderMode() == 3 && m_ringsRenderMode() > 1 );
    m_planetRingGeometries[4]->setEnabled( m_renderMode() == 3 && m_ringsRenderMode() > 1 );
    for ( auto& ring : m_planetRingGeometries ) {
        ring->update();
    }
    profiler( "Rings" );

    if ( m_axialTilt.dirty ) {
        m_planet.calcAnnualIllumination( 100, 100 );
        profiler("illumination");
    }

    m_renderTimes = profiler.results();
    m_redraw = false;

    m_cellsRenderMode.clear();

    if ( m_renderMode() == 1 ) {
        m_triCount = 0;
    } else if ( m_renderMode() == 2 ) {
        m_triCount = m_triangleGeometry->getSize();
    } else if ( m_renderMode() == 3 ) {
        m_triCount = m_planetGeometry->getSize();
    }
}

void MainWindow::generate() {
    m_geometryInvalid = true;
    m_redraw = true;
}

void MainWindow::updateText() {

    std::string format = "FPS %n\n"
                         "RenderMode: %n\n"
                         "WireframeMode: %n\n"
                         "PointMode: %n\n"
                         "ColorMode: %n\n"
                         "\n"
                         "Points: %n\n"
                         "Plates: %n/%n\n"
                         "Triangles: %n\n"
                         "Jitter: %n\n"
                         "CollisionThreshold: %n\n"
                         "Moisture: %n\n"
                         "AxialTilt: %n\n"
                         "TimeOfYear: %n\n"
                         "TimeOfDay: %n\n"
                         "\n";

    for ( const auto& time : m_planet.lastResults ) {
        format += time.first + ": %n\n";
    }
    format += "\n";
    for ( const auto& time : m_renderTimes ) {
        format += time.first + ": %n\n";
    }
    format += "\n";
    for ( const auto& time : m_lightingTimes ) {
        format += time.first + ": %n\n";
    }
    format += "\n";
    for ( const auto& time : m_pickingTimes ) {
        format += time.first + ": %n\n";
    }
    format += "\n";
    if ( m_renderMode() == 3 && m_highlight() >= 0 && m_highlight() < m_planet.cells.size() ) {
        format += "cell: %n\n"
                  "lat: %n\n"
                  "lon: %n\n"
                  "elevation: %n\n"
                  "compression: %n\n"
                  "neighbor: %n\n"
                  "plate: %n\n"
                  "plate elevation: %n\n"
                  "dMnt: %n\n"
                  "dCst: %n\n"
                  "dOcn: %n\n"
                  "illum: %n\n"
                  "annual: %n\n"
                  "temp: %n\n";
        for ( const auto& e : m_planet.cells[m_highlight()].edges ) {
            format += "n: %n b: %n\n";
        }
        format += "\n";
    }

    vl::Say say( format );
    say << fps()
        << m_renderMode()
        << m_wireFrameRenderMode()
        << m_pointsRenderMode()
        << m_cellsRenderMode()
        << m_planet.points.size()
        << m_planet.plates.size() << m_plateCount()
        << m_triCount
            << m_jitter()
            << m_collisionThreshold()
            << m_moisture()
            << m_axialTilt()
            << m_timeOfYear()
            << m_timeOfDay();

    for ( const auto& time : m_planet.lastResults ) {
        say << time.second;
    }

    for ( const auto& time : m_renderTimes ) {
        say << time.second;
    }

    for ( const auto& time : m_lightingTimes ) {
        say << time.second;
    }

    for ( const auto& time : m_pickingTimes ) {
        say << time.second;
    }
    if ( m_renderMode() == 3 && m_highlight() >= 0 && m_highlight() < m_planet.cells.size() ) {
        const auto& c = m_planet.cells[m_highlight()];
        say
                << c.point
                << -(m_planet.coords[c.point].x() * vl::dRAD_TO_DEG - 90)
                << (m_planet.coords[c.point].y() * vl::dRAD_TO_DEG)
                << c.elevation
                << c.compression
                << c.r
                << c.plate
                << m_planet.plates[c.plate].elevation
            << c.dMnt
            << c.dCst
            << c.dOcn
            << c.illumination
            << c.annualIllumination
            << c.temperature;

        for ( const auto& e : m_planet.cells[m_highlight()].edges ) {
            say << e.neighbor << e.bearing * vl::dRAD_TO_DEG;
        }
    }

    m_text->setText( say );

}

void MainWindow::updateGraphs() {
    if ( m_pickingActive() && m_highlight() >= 0 && m_highlight() < m_planet.cells.size() ) {
        m_dailyGraph->setData( m_planet.getDailyIllumination( m_highlight(), m_timeOfYear(), 100 ) );
        m_annualGraph->setData( m_planet.getAnnualIllumination( m_highlight(), 100 ) );
    }
    m_dailyGraph->setMarkerPosition( m_timeOfDay() );
    m_annualGraph->setMarkerPosition( m_timeOfYear() );
}


