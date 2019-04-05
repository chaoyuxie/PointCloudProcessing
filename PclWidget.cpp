#include "PclWidget.h"

PclWidget::PclWidget(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	//qvtkWidget = new QVTKWidget();
	//setCentralWidget(qvtkWidget);

	//qvtkWidget->update();

}

PclWidget::~PclWidget()
{
}

void PclWidget::Open()
{	// You might want to change "/home/" if you're not on an *nix platform
	QString filename = QFileDialog::getOpenFileName(this, tr("Open point cloud"), "/home/", tr("Point cloud data (*.pcd *.ply)"));

	PCL_INFO("File chosen: %s\n", filename.toStdString().c_str());
	PointCloudT::Ptr cloud_tmp(new PointCloudT);

	if (filename.isEmpty())
		return;

	int return_status;
	if (filename.endsWith(".pcd", Qt::CaseInsensitive))
		return_status = pcl::io::loadPCDFile(filename.toStdString(), *cloud_tmp);
	else
		return_status = pcl::io::loadPLYFile(filename.toStdString(), *cloud_tmp);

	if (return_status != 0)
	{
		PCL_ERROR("Error reading point cloud %s\n", filename.toStdString().c_str());
		return;
	}

	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloud_tmp->is_dense)
		pcl::copyPointCloud(*cloud_tmp, *cloud);
	else
	{
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud, vec);
	}

	//colorCloudDistances();
	viewer->updatePointCloud(cloud, "cloud");
	viewer->resetCamera();
	//update();
	qvtkWidget->update();

}


