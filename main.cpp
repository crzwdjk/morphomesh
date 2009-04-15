/**<!-------------------------------------------------------------------->
   @file   main.cpp
   @author Travis Fischer (fisch0920@gmail.com)
   @date   Spring 2009
   <!-------------------------------------------------------------------->**/

#include "Frontend.h"
#include <MiltonApp.h>

int main(int argc, char **argv) {
   MiltonApp app(argc, argv, "Morphomesh");
   
   {
     
     OpenGLCanvas *canvas = new OpenGLCanvas(false);
     
     if (argc < 2) {
       canvas->registerInteractionListener(new Frontend(canvas));
     } else {
       const std::string meshFile = argv[1];
       canvas->registerInteractionListener(new Frontend(canvas, meshFile));
     }

     app.addCanvas(canvas);
   }
   
   return app.exec();
}

