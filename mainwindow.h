#pragma once
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
#include <iostream>
#include "MyCloud.h"
#include "Tools.h"

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLCDNumber>
#include "ui_mainwindow.h"
#include "QVTKWidget.h"

#include <QFileDialog>
#include "QVTKWidget.h"
#include <QUrl>
#include <QMessageBox>
#include <QString>

// Point Cloud Library
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>

// Boost
#include <boost/math/special_functions/round.hpp>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class ViewWidget;
class QImage;
class QPainter;
class QRect;
class PclWidget;
QT_END_NAMESPACE

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = Q_NULLPTR);

private:
	Ui::MainWindowClass ui;
	MyCloud mycloud;
	std::vector<MyCloud> mycloud_vec;
	long total_points = 0;
public:
	QMenu		*menu_file_;
	QMenu		*menu_edit_;
	QMenu		*menu_help_;
	QToolBar	*toolbar_file_;
	//QAction		*action_init_;
	QAction		*action_about_;
	QAction		*action_new_;
	QAction		*action_open_;
	QAction		*action_add_;
	QAction		*action_downsample_;
	QAction		*action_save_;
	QAction		*action_saveas_;
	QAction		*action_random_color_;
	QSlider		*horizontalSlider_R;
	QSlider		*horizontalSlider_G;
	QSlider		*horizontalSlider_B;
	QSlider		*horizontalSlider_p;
	QLCDNumber	*lcdNumber_p;
	QLCDNumber	*lcdNumber_R;
	QLCDNumber	*lcdNumber_G;
	QLCDNumber	*lcdNumber_B;

	PclWidget	*pclwidget_;
	QVTKWidget	*qvtkWidget;
	QWidget		*pclwidget__;

protected:
	pcl::visualization::PCLVisualizer::Ptr viewer;
	PointCloudT::Ptr cloud;

	unsigned int red;
	unsigned int green;
	unsigned int blue;
private:
	void CreateAction();
	void CreateMenus();
	void CreateToolBars();
	void CreateStatusBar();
	void ActionAbout();
	public Q_SLOTS:
	void Open();
	void setA(unsigned int a);
	void RandomColor();
	void redSliderValueChanged(int value);
	void greenSliderValueChanged(int value);
	void blueSliderValueChanged(int value);
	void pSliderValueChanged(int value);
	void RGBsliderReleased();
	void Add();
	void showPointcloudAdd();
};
