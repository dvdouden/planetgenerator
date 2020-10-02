
#include <vlCore/VisualizationLibrary.hpp>
#include <vlSDL/SDLWindow.hpp>
#include <vlCore/Colors.hpp>
#include "MouseListener.h"
#include "MainWindow.h"

using namespace vl;

int main( int argc, char* args[] ) {
    /* init Visualization Library */
    VisualizationLibrary::init();

    /* setup the OpenGL context format */
    OpenGLContextFormat format;
    format.setDoubleBuffer( true );
    format.setRGBABits( 8, 8, 8, 0 );
    format.setDepthBufferBits( 24 );
    format.setStencilBufferBits( 8 );
    format.setFullscreen( false );
    //format.setMultisampleSamples(16);
    //format.setMultisample(true);

    /* create the applet to be run */

    MainWindow* w = new MainWindow;
    vl::ref<vl::Applet> applet = w;
    applet->initialize();
    //applet->setTrackball( new MouseListener( w ) );
    //applet->trackball()->setCamera( applet->rendering()->as<vl::Rendering>()->camera() );

    /* create a native SDL window */
    ref<vlSDL::SDLWindow> sdl_window = new vlSDL::SDLWindow;
    /* bind the applet so it receives all the GUI events related to the OpenGLContext */
    sdl_window->addEventListener( applet.get() );
    /* target the window so we can render on it */
    applet->rendering()->as<Rendering>()->renderer()->setFramebuffer( sdl_window->framebuffer() );

    /* Initialize the OpenGL context and window properties */
    int x = 0;
    int y = 0;
    int width = 1024;
    int height = 768;
    sdl_window->initSDLWindow( "Sphere rendering using Visualization Library", format, x, y, width, height );

    /* run SDL message loop */
    vlSDL::messageLoop();

    /* deallocate the window with all the OpenGL resources before shutting down Visualization Library */
    sdl_window = NULL;

    /* shutdown Visualization Library */
    VisualizationLibrary::shutdown();

    return 0;
}
// Have fun!
