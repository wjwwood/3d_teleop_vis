#include "ui_octree_vis_widget.h"
#include "octree_vis_widget/octree_vis_widget.h"

#include <QDebug>
 
#include <vtkPointSource.h>
#include <vtkOctreePointLocator.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkProperty.h>

// Constructor
OctreeVisWidget::OctreeVisWidget() {
  this->ui = new Ui_OctreeVisWidget;
  this->ui->setupUi(this);

  // Create a point cloud
  vtkSmartPointer<vtkPointSource> pointSource =
    vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetRadius(4);
  pointSource->SetNumberOfPoints(1000);
  pointSource->Update();

  vtkSmartPointer<vtkPolyDataMapper> pointsMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  pointsMapper->SetInputConnection(pointSource->GetOutputPort());

  vtkSmartPointer<vtkActor> pointsActor =
    vtkSmartPointer<vtkActor>::New();
  pointsActor->SetMapper(pointsMapper);
  pointsActor->GetProperty()->SetInterpolationToFlat();

  // Create the tree
  octree = vtkSmartPointer<vtkOctreePointLocator>::New();
  octree->SetDataSet(pointSource->GetOutput());
  octree->BuildLocator();

  polydata = vtkSmartPointer<vtkPolyData>::New();
  octree->GenerateRepresentation(0, polydata);

  vtkSmartPointer<vtkPolyDataMapper> octreeMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  octreeMapper->SetInputConnection(polydata->GetProducerPort());

  vtkSmartPointer<vtkActor> octreeActor =
    vtkSmartPointer<vtkActor>::New();
  octreeActor->SetMapper(octreeMapper);
  octreeActor->GetProperty()->SetInterpolationToFlat();
  octreeActor->GetProperty()->SetRepresentationToWireframe();
 
  // VTK Renderer
  renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(pointsActor);
  renderer->AddActor(octreeActor);
 
  // VTK/Qt wedded
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
 
  // Set up action signals and slots
  connect(this->ui->actionExit, SIGNAL(triggered()), this, SLOT(slotExit()));
  connect(this->ui->octree_lvl_slider, SIGNAL(sliderMoved(int)), this, SLOT(updateOctreeLevel(int)));

  this->ui->octree_lvl_slider->setMaximum(octree->GetLevel());
}
 
void OctreeVisWidget::updateOctreeLevel(int level) {
    this->octree->GenerateRepresentation(level, this->polydata);
    this->renderer->Render();
    this->ui->qvtkWidget->GetRenderWindow()->Render();
}

void OctreeVisWidget::slotExit() 
{
  qApp->exit();
}
