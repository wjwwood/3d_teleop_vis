#include <QApplication>
#include "octree_vis_widget/octree_vis_widget.h"
 
int main( int argc, char** argv )
{
  // QT Stuff
  QApplication app( argc, argv );
 
  OctreeVisWidget octree_vis_widget;
  octree_vis_widget.show();
 
  return app.exec();
}