#ifndef OCTREE_VIS_WIDGET_H
#define OCTREE_VIS_WIDGET_H

#include <QWidget>

#include "vtkSmartPointer.h"
#include <vtkPolyData.h>
#include <vtkOctreePointLocator.h>
#include <vtkRenderer.h>

// Forward Qt class declarations
class Ui_OctreeVisWidget;
 
class OctreeVisWidget : public QWidget {
  Q_OBJECT
public:

  // Constructor/Destructor
  OctreeVisWidget();
  ~OctreeVisWidget() {}

public slots:

  virtual void slotExit();
  virtual void updateOctreeLevel(int);

protected:

protected slots:

private:
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkOctreePointLocator> octree;
  vtkSmartPointer<vtkRenderer> renderer;

  // Designer form
  Ui_OctreeVisWidget *ui;
};
 
#endif // OCTREE_VIS_WIDGET_H
