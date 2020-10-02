#include <vlCore/VisualizationLibrary.hpp>
#include <vlGLUT/GLUTWindow.hpp>
#include <vlCore/Colors.hpp>
#include "MouseListener.h"
#include "MainWindow.h"

using namespace vl;

int main( int argc, char* argv[] ) {
    /* init GLUT */
    int pargc = argc;
    glutInit( &pargc, argv );

    /* init Visualization Library */
    VisualizationLibrary::init();

    /* install Visualization Library shutdown function */
    atexit( vlGLUT::atexit_visualization_library_shutdown );

    /* setup the OpenGL context format */
    OpenGLContextFormat format;
    format.setDoubleBuffer( true );
    format.setRGBABits( 8, 8, 8, 0 );
    format.setDepthBufferBits( 24 );
    format.setStencilBufferBits( 8 );
    format.setMultisampleSamples( 16 );
    format.setMultisample( false );
    format.setFullscreen( false );
    //format.setOpenGLProfile( GLP_Core );
    //format.setVersion( 3, 3 );

    /* create the applet to be run */

    MainWindow* w = new MainWindow;
    vl::ref<vl::Applet> applet = w;
    applet->initialize();
    //applet->setTrackball( new MouseListener( w ) );
    //applet->trackball()->setCamera( applet->rendering()->as<vl::Rendering>()->camera() );



    /* create a native GLUT window */
    ref<vlGLUT::GLUTWindow> glut_window = new vlGLUT::GLUTWindow;
    /* bind the applet so it receives all the GUI events related to the OpenGLContext */
    glut_window->addEventListener( applet.get() );
    /* target the window so we can render on it */
    applet->rendering()->as<Rendering>()->renderer()->setFramebuffer( glut_window->framebuffer() );

    /* Initialize the OpenGL context and window properties */
    int x = 0;
    int y = 0;
    int width = 800;
    int height = 600;
    glut_window->initGLUTWindow( "Sphere rendering using Visualization Library", format, x, y, width, height );

    /* ... you can open more than one GLUT window! */

    /* enter the GLUT main loop */
    glutMainLoop();

    /* this point is never reached since glutMainLoop() never returns! */

    return 0;
}
// Have fun!
