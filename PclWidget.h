#pragma once
//#include <vtkAutoInit.h> 
//VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkInteractionStyle);
#include <iostream>

#include <QWidget>
#include "ui_PclWidget.h"
#include <QFileDialog>
#include "QVTKWidget.h"

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PclWidget : public QWidget
{
	Q_OBJECT

public:
	PclWidget(QWidget *parent = Q_NULLPTR);
	~PclWidget();
	QVTKWidget *qvtkWidget;
	PclWidget	*pclwidget_;
protected:
	pcl::visualization::PCLVisualizer::Ptr viewer;
	PointCloudT::Ptr cloud;
private:
	Ui::PclWidget ui;
	public Q_SLOTS:
	void Open();
	//void Init();
};
