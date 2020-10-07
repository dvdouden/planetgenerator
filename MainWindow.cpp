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

    m_sunCircle = new GradientRingGeometry();
    m_sceneManager->tree()->addActor( m_sunCircle.get(), m_effect.get(), m_sunRotationTransform.get() )->setObjectName( "Sun Path");
    m_sunCircle->setEnabled( true );

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
    m_sceneManager->tree()->addActor( m_planetGeometry.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "Planet" );
    m_plateOriginGeometry = new PlateOriginGeometry( m_planet );
    m_sceneManager->tree()->addActor( m_plateOriginGeometry.get(), m_effect.get(), m_planetTransform.get() )->setObjectName( "PlateOrigins" );
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

    generate();
    updateGeometry();

    // set up key bindings for each mode
    m_parameters.resize( 10, std::vector<param_ptr>() );
    m_keyBindings.resize( 10 );

    auto* modeBinding = new constantParameterBinding<int>( m_planet.phase, true );
    modeBinding->addKey( vl::Key_1, 1 );
    modeBinding->addKey( vl::Key_2, 2 );
    modeBinding->addKey( vl::Key_3, 3 );
    modeBinding->addKey( vl::Key_4, 4 );
    modeBinding->addKey( vl::Key_5, 5 );
    modeBinding->addKey( vl::Key_6, 6 );
    modeBinding->addKey( vl::Key_7, 7 );
    modeBinding->addKey( vl::Key_8, 8 );
    modeBinding->addKey( vl::Key_9, 9 );
    modeBinding->addKey( vl::Key_0, 0 );
    param_ptr modePtr( modeBinding );

    // set up common key bindings
    for ( int mode = 0 ; mode < 10; ++mode ) {
        m_parameters[mode].emplace_back( modePtr );
    }

    param_ptr pointsBinding( new repeatingParameterBinding<int>( m_planet.pointCount, 8, 1024 * 1024, 1, vl::Key_Right, vl::Key_Left, true ) );
    param_ptr fastPointsBinding( new repeatingParameterBinding<int>( m_planet.pointCount, 8, 1024 * 1024, 10, vl::Key_Up, vl::Key_Down, true ) );
    param_ptr reallyFastPointsBinding( new multiplyingParameterBinding<int>( m_planet.pointCount, 8, 1024 * 1024, 2, vl::Key_PageUp, vl::Key_PageDown, true ) );

    param_ptr renderPlanetAxis( new boolParameterBinding( m_renderPlanetAxis, vl::Key_A, false ));
    param_ptr jitter( new parameterBinding<float>( m_planet.jitter, 0.0f, 360.0f, 1.0f, vl::Key_Plus, vl::Key_Minus, true ));
    param_ptr stereo( new parameterBinding<int>( m_stereoFactor, 0, 100, 1, vl::Key_RightBracket, vl::Key_LeftBracket, false ));
    param_ptr pointsRenderMode( new parameterBinding<int>( m_pointsRenderMode, 0, 9, 1, vl::Key_P, vl::Key_O, false ));
    param_ptr renderCentroids( new boolParameterBinding( m_renderCentroids, vl::Key_C, false ));
    param_ptr plates( new parameterBinding<int>( m_planet.plateCount, 0, 10000, 1, vl::Key_Insert, vl::Key_Delete, true ));
    param_ptr useCentroids( new boolParameterBinding( m_planet.useCentroids, vl::Key_BackSlash, true ));
    param_ptr normalizeCentroids( new boolParameterBinding( m_planet.normalizeCentroids, vl::Key_N, true ));
    param_ptr cellsRenderMode( new parameterBinding<int>( m_cellsRenderMode, 0, 9, 1, vl::Key_R, vl::Key_E, false ));
    param_ptr wireFrameRenderMode( new parameterBinding<int>( m_wireFrameRenderMode, 0, 9, 1, vl::Key_W, vl::Key_Q, false ));
    param_ptr ocean( new parameterBinding<int>( m_planet.ocean, 0, 100, 1, vl::Key_J, vl::Key_H, true ));
    param_ptr renderPlateVectors( new boolParameterBinding( m_renderPlateVectors, vl::Key_V, false ));
    param_ptr renderPlateOrigins( new boolParameterBinding( m_renderPlateOrigins, vl::Key_C, false ));
    param_ptr collisionThreshold( new parameterBinding<float>( m_planet.collisionThreshold, 0.0f, 4.0f, 0.05f, vl::Key_Period, vl::Key_Comma, true ));
    param_ptr pickingActive( new boolParameterBinding( m_pickingActive, vl::Key_Space, false ));

    param_ptr ringsRenderMode( new parameterBinding<int>( m_ringsRenderMode, 0, 9, 1, vl::Key_X, vl::Key_Z, false ));
    param_ptr moisture( new parameterBinding<int>( m_planet.moisture, 0, 100, 1, vl::Key_Quote, vl::Key_Semicolon, true ));
    param_ptr axialTilt( new parameterBinding<float>( m_planet.axialTilt, 0.0f, 180.0f, 0.5f, vl::Key_Home, vl::Key_End, false ));
    param_ptr timeOfYear( new parameterBinding<float>( m_timeOfYear, 0.0f, 1.0f, 0.0625f, vl::Key_L, vl::Key_K, false ));
    param_ptr timeOfDayPaused( new boolParameterBinding( m_timeOfDayPaused, vl::Key_T, false ));
    param_ptr timeOfYearPaused( new boolParameterBinding( m_timeOfYearPaused, vl::Key_Y, false ));

    param_ptr noiseIntensity( new parameterBinding<float>( m_planet.noiseIntensity, 0.0f, 1.0f, 0.05f, vl::Key_Y, vl::Key_T, true ));
    param_ptr noiseOctaves( new parameterBinding<int>( m_planet.noiseOctaves, 0, 8, 1, vl::Key_H, vl::Key_G, true ));
    param_ptr noiseScale( new parameterBinding<float>( m_planet.noiseScale, 0.0f, 20.0f, 0.5f, vl::Key_N, vl::Key_B, true ));


    // mode 1: points
    m_parameters[1].emplace_back( jitter );
    m_parameters[1].emplace_back( stereo );
    m_parameters[1].emplace_back( pointsRenderMode );
    m_parameters[1].emplace_back( pointsBinding );
    m_parameters[1].emplace_back( fastPointsBinding );
    m_parameters[1].emplace_back( reallyFastPointsBinding );

    // mode 2: triangles
    m_parameters[2].emplace_back( jitter );
    m_parameters[2].emplace_back( stereo );
    m_parameters[2].emplace_back( pointsBinding );
    m_parameters[2].emplace_back( fastPointsBinding );
    m_parameters[2].emplace_back( reallyFastPointsBinding );
    m_parameters[2].emplace_back( renderPlanetAxis );

    // mode 3: cells
    m_parameters[3].emplace_back( jitter );
    m_parameters[3].emplace_back( renderCentroids );
    m_parameters[3].emplace_back( useCentroids );
    m_parameters[3].emplace_back( normalizeCentroids );
    m_parameters[3].emplace_back( cellsRenderMode );
    m_parameters[3].emplace_back( wireFrameRenderMode );
    m_parameters[3].emplace_back( pickingActive );
    m_parameters[3].emplace_back( pointsBinding );
    m_parameters[3].emplace_back( fastPointsBinding );
    m_parameters[3].emplace_back( reallyFastPointsBinding );
    m_parameters[3].emplace_back( renderPlanetAxis );

    // mode 4: plates
    m_parameters[4].emplace_back( plates );
    m_parameters[4].emplace_back( cellsRenderMode );
    m_parameters[4].emplace_back( wireFrameRenderMode );
    m_parameters[4].emplace_back( ocean );
    m_parameters[4].emplace_back( pickingActive );
    m_parameters[4].emplace_back( renderPlateOrigins );
    m_parameters[4].emplace_back( renderPlanetAxis );

    // mode 5: tectonics / heightmap
    m_parameters[5].emplace_back( plates );
    m_parameters[5].emplace_back( cellsRenderMode );
    m_parameters[5].emplace_back( wireFrameRenderMode );
    m_parameters[5].emplace_back( renderPlateVectors );
    m_parameters[5].emplace_back( collisionThreshold );
    m_parameters[5].emplace_back( pickingActive );
    m_parameters[5].emplace_back( noiseIntensity );
    m_parameters[5].emplace_back( noiseOctaves );
    m_parameters[5].emplace_back( noiseScale );
    m_parameters[5].emplace_back( renderPlanetAxis );

    // mode 6: climate
    m_parameters[6].emplace_back( ringsRenderMode );
    m_parameters[6].emplace_back( moisture );
    m_parameters[6].emplace_back( axialTilt );
    m_parameters[6].emplace_back( timeOfYear );
    m_parameters[6].emplace_back( timeOfDayPaused );
    m_parameters[6].emplace_back( timeOfYearPaused );
    m_parameters[6].emplace_back( renderPlanetAxis );

    // verify key bindings
    for ( int i = 0; i < 10; ++i ) {
        std::map<vl::EKey, param_ptr>& keys = m_keyBindings[i];
        for ( auto& b : m_parameters[i] ) {
            for ( auto key : b->getKeys() ) {
                auto it = keys.find( key );
                if ( it != keys.end() ) {
                    printf( "[%d] DUPLICATE KEY BINDING for %s, already bound by %s!\n", i, b->name().c_str(), (*it).second->name().c_str() );
                }
                keys[key] = b;
            }
        }
    }

    updateText();
    updateGraphs();
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
    if ( m_keyBindings[m_planet.phase()].find( key ) != m_keyBindings[m_planet.phase()].end() ) {
        m_keyBindings[m_planet.phase()][key]->pressed( key );
        regenerate = regenerate || m_keyBindings[m_planet.phase()][key]->generate;
    }
    else {
        Applet::keyPressEvent( ch, key );
    }

    if ( regenerate ) {
        generate();
    }
}

void MainWindow::keyPressedEvent() {
    bool regenerate = false;
    for ( vl::EKey key : m_pressedKeys ) {
        if ( m_keyBindings[m_planet.phase()].find( key ) != m_keyBindings[m_planet.phase()].end() ) {
            m_keyBindings[m_planet.phase()][key]->held( key );
            regenerate = regenerate || m_keyBindings[m_planet.phase()][key]->generate;
        }
    }

    if ( regenerate ) {
        generate();
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

    if ( m_pickingActive() && m_planetGeometry->isEnabled() && m_mousePosition.dirty && !m_geometryInvalid ) {
        m_mousePosition.clear();
        Profiler profiler( "Picking" );
        int x = m_mousePosition().x();
        int y = m_mousePosition().y();
        y = openglContext()->height() - y;
        m_highlight = m_planetIntersector.intersect( rendering()->as<vl::Rendering>()->camera()->computeRay(x,y) );

        profiler("Check intersections" );
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
        m_timeOfDay = m_timeOfDay() + m_deltaTimeOfDay;
        m_timeOfDay = (float)modf( m_timeOfDay(), &tmp );
        m_sunPositionDirty = true;
    }
    if ( !m_timeOfYearPaused() ) {
        m_deltaTimeOfYear = (float)deltaTime / secondsPerYear;
        m_timeOfYear = m_timeOfYear() + m_deltaTimeOfYear;
        m_timeOfYear = (float)modf( m_timeOfYear(), &tmp );
        m_sunPositionDirty = true;
    }

    if ( m_planet.axialTilt.dirty ) {
        m_sunPositionDirty = true;
        m_planetRingGeometries[0]->setLatitude( 90 - m_planet.axialTilt() );
        m_planetRingGeometries[1]->setLatitude( m_planet.axialTilt() );
        // equator doesn't change (duh)
        m_planetRingGeometries[3]->setLatitude( -m_planet.axialTilt() );
        m_planetRingGeometries[4]->setLatitude( -90 + m_planet.axialTilt() );
    }

    if ( m_sunPositionDirty ) {
        double radTOY = (m_timeOfYear() + 0.5) * 2 *
                        vl::dPi; // add a half, we'd like the year to start with winter in the northern hemisphere
        vl::mat4 tiltMtx = vl::mat4::getRotation( m_planet.axialTilt(), sin( radTOY ) * vl::dRAD_TO_DEG, 0,
                                                  cos( radTOY ) * vl::dRAD_TO_DEG );

        double degTOD = m_timeOfDay() * 360;
        vl::mat4 rotMtx = vl::mat4::getRotation( -degTOD, 0, 1, 0 );

        m_sunTransform->setLocalMatrix( tiltMtx );
        m_sunRotationTransform->setLocalMatrix( rotMtx );
        m_sunRotationTransform->computeWorldMatrix();
        m_sunRay = m_sunTransform->worldMatrix() * vl::fvec3( 1, 0, 0 );

        m_sunCircle->setLatitude( m_planet.axialTilt() * cos( radTOY ) );
    }

    if ( (m_cellsRenderMode() == 6 || m_cellsRenderMode() == 8) && m_sunPositionDirty ) {
        Profiler profiler("lighting and heat");
        m_planet.calcLight( m_sunRay );
        profiler( "light" );
        m_planet.updateTemperature( m_deltaTimeOfDay );
        profiler( "temperature" );
        m_lightingTimes = profiler.results();
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

    if ( m_renderPlanetAxis.dirty ) {
        m_planetAxis->setEnabled( m_renderPlanetAxis() );
        m_planetAxis->update();
        m_renderPlanetAxis.clear();
    }

    updateGeometry();

    updateText();
    updateGraphs();
    m_sunPositionDirty = false;
    m_planet.axialTilt.clear();
}


void MainWindow::updateGeometry() {
    Profiler profiler( "Renderer" );

    if ( m_planet.phase.dirty ) {
        m_planetGeometry->markColorsDirty();
    }

    m_planet.generate();
    m_geometryInvalid = false;
    profiler( "generator", m_planet.lastResults );

    if ( m_planet.pointsDirty ) {
        m_pointsGeometry->markGeometryInvalid();
    }
    if ( m_planet.trianglesDirty ) {
        m_triangleGeometry->markGeometryInvalid();
        m_centroidsGeometry->markGeometryInvalid();
    }
    if ( m_planet.cellsDirty ) {
        m_planetGeometry->markGeometryInvalid();
        m_cellLines->markGeometryInvalid();
    }
    if ( m_planet.platesDirty ) {
        m_borderLines->markGeometryInvalid();
        m_cellLines->markGeometryInvalid();
        m_cellVectors->markGeometryInvalid();
        m_plateOriginGeometry->markGeometryInvalid();
        m_planetGeometry->markColorsDirty();
    }
    if ( m_planet.cellColorsDirty ) {
        m_planetGeometry->markColorsDirty();
    }
    m_planet.clearDirtyFlags();

    m_pointsGeometry->setEnabled( m_planet.phase() == 1 );
    if ( m_pointsGeometry->isEnabled() ) {
        m_pointsGeometry->update();
        profiler( "Points" );
    }

    m_triangleGeometry->setEnabled( m_planet.phase() == 2 );
    if ( m_triangleGeometry->isEnabled() ) {
        m_triangleGeometry->update();
        profiler( "Triangles" );
    }

    m_centroidsGeometry->setEnabled( m_renderCentroids() );
    if ( m_centroidsGeometry->isEnabled() ) {
        m_centroidsGeometry->update();
        profiler( "Centroids" );
    }

    m_planetGeometry->setEnabled( m_planet.phase() >= 3 );
    if( m_planetGeometry->isEnabled() ) {
        m_planetGeometry->update();
        profiler( "Planet" );
    }

    m_borderLines->setEnabled( m_planet.phase() >= 4 && m_wireFrameRenderMode() > 0 );
    if ( m_borderLines->isEnabled() ) {
        m_borderLines->update();
        profiler( "Plateborders" );
    }

    m_cellLines->setIncludeBorders( m_planet.phase() == 3 );
    m_cellLines->setEnabled( m_planetGeometry->isEnabled() && m_wireFrameRenderMode() > 1 );
    if ( m_cellLines->isEnabled() ) {
        m_cellLines->update();
        profiler( "Cellborders" );
    }

    m_plateOriginGeometry->setEnabled( m_planet.phase() == 4 && m_renderPlateOrigins() );
    if ( m_plateOriginGeometry->isEnabled() ) {
        m_plateOriginGeometry->update();
        profiler( "PlateOrigins" );
    }

    m_cellVectors->setEnabled( m_planet.phase() == 5 && m_renderPlateVectors() );
    if ( m_cellVectors->isEnabled() ) {
        m_cellVectors->update();
        profiler( "CellVectors" );
    }

    m_sunCircle->setEnabled( m_planet.phase() == 6 );
    if ( m_sunCircle->isEnabled() ) {
        m_sunCircle->update();
        profiler( "SunCircle" );
    }


    m_planetRingGeometries[0]->setEnabled( m_planet.phase() == 6 && m_ringsRenderMode() > 1 );
    m_planetRingGeometries[1]->setEnabled( m_planet.phase() == 6 && m_ringsRenderMode() > 1 );
    m_planetRingGeometries[2]->setEnabled( m_planet.phase() == 6 && m_ringsRenderMode() > 0 );
    m_planetRingGeometries[3]->setEnabled( m_planet.phase() == 6 && m_ringsRenderMode() > 1 );
    m_planetRingGeometries[4]->setEnabled( m_planet.phase() == 6 && m_ringsRenderMode() > 1 );
    if ( m_planetRingGeometries[2]->isEnabled() ) {
        for ( auto& ring : m_planetRingGeometries ) {
            ring->update();
        }
        profiler( "Rings" );
    }

    if ( m_planet.axialTilt.dirty ) {
        m_planet.calcAnnualIllumination( 100, 100 );
        profiler("illumination");
    }

    m_renderTimes = profiler.results();

    m_cellsRenderMode.clear();

    if ( m_planet.phase() == 1 ) {
        m_triCount = 0;
    } else if ( m_planet.phase() == 2 ) {
        m_triCount = m_triangleGeometry->getSize();
    } else if ( m_planetGeometry->isEnabled() ) {
        m_triCount = m_planetGeometry->getSize();
    }
    m_planet.phase.clear();
}

void MainWindow::generate() {
    m_geometryInvalid = true;
}

void MainWindow::updateText() {

    std::string format = "FPS %n\n";
    std::set<std::string> params;
    for ( auto& p : m_parameters[m_planet.phase()] ) {
        if ( params.find( p->name() ) == params.end() ) {
            format += p->name() + ": " + p->format() + "\n";
            params.insert( p->name() );
        }
    }
    format += "\n";

    format += "\n";
    for ( const auto& time : m_renderTimes ) {
        format += time.name + ": %n\n";
        for ( const auto& t2 : time.details ) {
            format += std::string("  " ) + t2.name + ": %n\n";
        }
    }
    format += "\n";
    for ( const auto& time : m_lightingTimes ) {
        format += time.name + ": %n\n";
    }
    format += "\n";
    for ( const auto& time : m_pickingTimes ) {
        format += time.name + ": %n\n";
    }
    format += "\n";
    if ( m_planet.phase() >= 3 && m_highlight() >= 0 && m_highlight() < m_planet.cells.size() ) {
        format += "cell: %n\n"
                  "lat: %n\n"
                  "lon: %n\n";
        if ( m_planet.phase() >= 4 ) {
            format += "plate: %n\n"
                      "plate type: %s\n"
                      "plate cells: %n\n";
        }
        if ( m_planet.phase() >= 5 ) {
            format += "elevation: %n\n";
        }
        if ( m_planet.phase() == 5 ) {
            format += "bearing: %n\n"
                      "convergent: %n\n"
                      "divergent: %n\n";
            for ( const auto& e : m_planet.cells[m_highlight()].edges ) {
                if ( e.plateBorder ) {
                    format += "n: %n edgeF: %n lenF: %n neighF: %n F: %n type: %s\n";
                }
            }
        }
        if ( m_planet.phase() == 6 ) {
            format += "illum: %n\n"
                      "annual: %n\n"
                      "temp: %n\n";
        }

        format += "\n";
    }

    vl::Say say( format );
    say << fps();
    params.clear();
    for ( auto& p : m_parameters[m_planet.phase()] ) {
        if ( params.find( p->name() ) == params.end() ) {
            p->say( say );
            params.insert( p->name() );
        }
    }

    for ( const auto& time : m_renderTimes ) {
        say << time.us;
        for ( const auto& t2 : time.details ) {
            say << t2.us;
        }
    }

    for ( const auto& time : m_lightingTimes ) {
        say << time.us;
    }

    for ( const auto& time : m_pickingTimes ) {
        say << time.us;
    }
    if ( m_planet.phase() >= 3 && m_highlight() >= 0 && m_highlight() < m_planet.cells.size() ) {
        const auto& c = m_planet.cells[m_highlight()];
        say << c.point
            << -(m_planet.coords[c.point].x() * vl::dRAD_TO_DEG - 90)
            << (m_planet.coords[c.point].y() * vl::dRAD_TO_DEG);
        if ( m_planet.phase() >= 4 ) {
            say << c.plate
                << (m_planet.plates[c.plate].oceanic ? "oceanic" : "continental")
                << m_planet.plates[c.plate].cellCount;
        }
        if ( m_planet.phase() >= 5 ) {
            say << c.elevation;
        }
        if ( m_planet.phase() == 5 ) {
            say << c.bearing * vl::dRAD_TO_DEG
                << c.convergentForce
                << c.divergentForce;
            for ( const auto& e : m_planet.cells[m_highlight()].edges ) {
                if ( e.plateBorder ) {
                    say << e.neighbor << e.edgeFactor << e.lengthFactor << e.neighborDirFactor << e.force << (e.convergent ? "convergent" : "divergent" );
                }
            }
        }
        if ( m_planet.phase() == 6 ) {
            say << c.illumination
                << c.annualIllumination
                << c.temperature;
        }
    }

    m_text->setText( say );

}

void MainWindow::updateGraphs() {
    m_dailyGraph->setEnabled( m_planet.phase() == 6 );
    m_annualGraph->setEnabled( m_planet.phase() == 6 );
    if ( m_planet.phase() == 6 ) {
        if ( m_pickingActive() && m_highlight() >= 0 && m_highlight() < m_planet.cells.size() ) {
            m_dailyGraph->setData( m_planet.getDailyIllumination( m_highlight(), m_timeOfYear(), 100 ) );
            m_annualGraph->setData( m_planet.getAnnualIllumination( m_highlight(), 100 ) );
        }
        m_dailyGraph->setMarkerPosition( m_timeOfDay() );
        m_annualGraph->setMarkerPosition( m_timeOfYear() );
    }
}


