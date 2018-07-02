#include <Viewer.h>
#include <MapDrawer.h>
#include <pangolin/pangolin.h>

using namespace SLAM;

Viewer::Viewer(Map* map) : mMap(map){
  mMapDrawer = new MapDrawer(mMap);
}

void
Viewer::run(){
  pangolin::CreateWindowAndBind("Main",640,480);
  glEnable(GL_DEPTH_TEST);

  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
      pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
  );

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
          .SetHandler(&handler);

  // UI WIDTH
  const int UI_WIDTH = 180;
          
  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
              
  // Safe and efficient binding of named variables.
  // Specialisations mean no conversions take place for exact types
  // and conversions between scalar types are cheap.
  pangolin::Var<bool> addButton("ui.Add Button",false,false);
  pangolin::Var<double> inputX("ui.X",0,false);
  pangolin::Var<double> inputY("ui.Y",0,false);
  pangolin::Var<double> inputZ("ui.Z",0,false);
      
  while( !pangolin::ShouldQuit() )
  {
      // Clear screen and activate view to render into
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      d_cam.Activate(s_cam);
      
      // Render OpenGL Cube
      mMapDrawer->drawMapPoints();

      // Swap frames and Process Events
      pangolin::FinishFrame();
  }
}