#include <Viewer.h>

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>

#include <MapDrawer.h>
#include <Map.h>
#include <MapPoint.h>


using namespace SLAM;

Viewer::Viewer(Map* map) : mMap(map){
  mMapDrawer = new MapDrawer(mMap);
}

void
Viewer::run(){
  pangolin::CreateWindowAndBind("Main",1024,768);
  glEnable(GL_DEPTH_TEST);

  // Define Projection and initial ModelView matrix
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024,768,420,420,320,240,0.2,100),
      pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
  );

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View& d_cam = pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
          .SetHandler(&handler);

  // UI WIDTH
  const int UI_WIDTH = 180;
          
  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
      
  pangolin::Renderable tree;
  tree.Add( std::make_shared<pangolin::Axis>() );
      
  d_cam.SetDrawFunction([&](pangolin::View& view){
      view.Activate(s_cam);
      tree.Render();
  });        
      
  // Safe and efficient binding of named variables.
  // Specialisations mean no conversions take place for exact types
  // and conversions between scalar types are cheap.
  pangolin::Var<bool> addButton("ui.Add Button",false,false);
  pangolin::Var<double> inputX("ui.X",0,-10,10);
  pangolin::Var<double> inputY("ui.Y",0,-10,10);
  pangolin::Var<double> inputZ("ui.Z",0,-10,10);
      
  while( !pangolin::ShouldQuit() )
  {
      // add new point from UI
      if( pangolin::Pushed(addButton) ){
        mMap->addMapPoint(new MapPoint((double) inputX,(double) inputY,(double) inputZ) );
      }
      
      // Clear screen and activate view to render into
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      d_cam.Activate(s_cam);
      glClearColor(1.0,1.0,1.0,1.0);
      
      // Render OpenGL Cube
      mMapDrawer->drawMapPoints();

      // Swap frames and Process Events
      pangolin::FinishFrame();
  }
}