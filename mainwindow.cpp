#include "mainwindow.h"
#include<qmessagebox.h>
#include"PclWidget.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	this->setWindowTitle("PCL viewer");
	pclwidget_ = new PclWidget();
	setCentralWidget(pclwidget_);
	resize(966, 599);
	// The default color
	red = 128;
	green = 128;
	blue = 128;

	horizontalSlider_R = new QSlider(pclwidget_);
	horizontalSlider_R->setObjectName(QStringLiteral("horizontalSlider_R"));
	horizontalSlider_R->setGeometry(QRect(30, 60, 160, 29));
	horizontalSlider_R->setMaximum(255);
	horizontalSlider_R->setValue(128);
	horizontalSlider_R->setOrientation(Qt::Horizontal);
	horizontalSlider_G = new QSlider(pclwidget_);
	horizontalSlider_G->setObjectName(QStringLiteral("horizontalSlider_G"));
	horizontalSlider_G->setGeometry(QRect(30, 140, 160, 29));
	horizontalSlider_G->setMaximum(255);
	horizontalSlider_G->setValue(128);
	horizontalSlider_G->setOrientation(Qt::Horizontal);
	horizontalSlider_B = new QSlider(pclwidget_);
	horizontalSlider_B->setObjectName(QStringLiteral("horizontalSlider_B"));
	horizontalSlider_B->setGeometry(QRect(30, 220, 160, 29));
	horizontalSlider_B->setMaximum(255);
	horizontalSlider_B->setValue(128);
	horizontalSlider_B->setOrientation(Qt::Horizontal);
	lcdNumber_R = new QLCDNumber(pclwidget_);
	lcdNumber_R->setObjectName(QStringLiteral("lcdNumber_R"));
	lcdNumber_R->setGeometry(QRect(200, 50, 81, 41));
	lcdNumber_R->setDigitCount(3);
	lcdNumber_R->setSegmentStyle(QLCDNumber::Flat);
	lcdNumber_R->setProperty("intValue", QVariant(128));
	lcdNumber_G = new QLCDNumber(pclwidget_);
	lcdNumber_G->setObjectName(QStringLiteral("lcdNumber_G"));
	lcdNumber_G->setGeometry(QRect(200, 130, 81, 41));
	lcdNumber_G->setDigitCount(3);
	lcdNumber_G->setSegmentStyle(QLCDNumber::Flat);
	lcdNumber_G->setProperty("intValue", QVariant(128));
	lcdNumber_B = new QLCDNumber(pclwidget_);
	lcdNumber_B->setObjectName(QStringLiteral("lcdNumber_B"));
	lcdNumber_B->setGeometry(QRect(200, 210, 81, 41));
	lcdNumber_B->setDigitCount(3);
	lcdNumber_B->setSegmentStyle(QLCDNumber::Flat);
	lcdNumber_B->setProperty("intValue", QVariant(128));
	horizontalSlider_p = new QSlider(pclwidget_);
	horizontalSlider_p->setObjectName(QStringLiteral("horizontalSlider_p"));
	horizontalSlider_p->setGeometry(QRect(30, 320, 160, 29));
	horizontalSlider_p->setMinimum(1);
	horizontalSlider_p->setMaximum(6);
	horizontalSlider_p->setValue(2);
	horizontalSlider_p->setOrientation(Qt::Horizontal);

	lcdNumber_p = new QLCDNumber(pclwidget_);
	lcdNumber_p->setObjectName(QStringLiteral("lcdNumber_p"));
	lcdNumber_p->setGeometry(QRect(200, 310, 81, 41));
	lcdNumber_p->setDigitCount(1);
	lcdNumber_p->setSegmentStyle(QLCDNumber::Flat);
	lcdNumber_p->setProperty("intValue", QVariant(2));

	qvtkWidget = new QVTKWidget(pclwidget_);
	qvtkWidget->setGeometry(QRect(300, 10, 640, 480));
	cloud.reset(new PointCloudT);
	cloud->resize(1);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	
	viewer->setupInteractor(qvtkWidget->GetInteractor(), qvtkWidget->GetRenderWindow());
	viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
	qvtkWidget->update();
	//setCentralWidget(qvtkWidget);
	//qvtkWidget = new QVTKWidget(pclwidget_);
	//qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
	//qvtkWidget->setGeometry(QRect(300, 10, 640, 480));
	CreateAction();
	CreateMenus();
	CreateToolBars();
	CreateStatusBar();
	viewer->addPointCloud(cloud, "cloud");
	pSliderValueChanged(2);
	viewer->resetCamera();
	qvtkWidget->update();
}
void MainWindow::CreateAction()
{

	action_about_ = new QAction(QIcon("Resources\\images\\about.png"),tr("&About"), this);
	action_about_->setStatusTip(tr("About MainWindow..."));
	connect(action_about_, &QAction::triggered, this, &MainWindow::ActionAbout);

	action_new_ = new QAction(QIcon("Resources\\images\\new.png"), tr("&New"), this);
	action_new_->setShortcut(QKeySequence::New);
	action_new_->setStatusTip(tr("Create a new file"));

	action_open_ = new QAction(QIcon("Resources\\images\\open.png"), tr("&Open..."), this);
	action_open_->setShortcuts(QKeySequence::Open);
	action_open_->setStatusTip(tr("Open an existing file"));
	connect(action_open_, SIGNAL(triggered()), this, SLOT(Open()));

	action_save_ = new QAction(QIcon("Resources\\images\\save.png"), tr("&Save"), this);
	action_save_->setShortcuts(QKeySequence::Save);
	action_save_->setStatusTip(tr("Save the document to disk"));

	action_saveas_ = new QAction(tr("Save &As..."), this);
	action_saveas_->setShortcuts(QKeySequence::SaveAs);
	action_saveas_->setStatusTip(tr("Save the document under a new name"));
	//connect(action_saveas_, SIGNAL(triggered()), pclwidget_, SLOT(SaveAs()));

	action_add_ = new QAction(QIcon("Resources\\images\\add.png"),tr("Add"), this);
	//action_add_->setShortcuts(tr("ctrl+alt+a"));
	action_add_->setStatusTip(tr("Add some files"));
	connect(action_add_, SIGNAL(triggered()), this, SLOT(Add()));

	action_downsample_ = new QAction(tr("DownSample"), this);
	//action_downsample_->setShortcuts(tr("ctrl+alt+a"));
	action_downsample_->setStatusTip(tr("Downsample for this point cloud"));
	//connect(action_downsample_, SIGNAL(triggered()), pclwidget_, SLOT(DownSample()));

	action_random_color_ = new QAction(tr("Random_Color"), this);
	//action_random_color_->setShortcuts(QKeySequence::SaveAs);
	action_random_color_->setStatusTip(tr("Random select color for the point cloud"));
	connect(action_random_color_, SIGNAL(triggered()), this, SLOT(RandomColor()));

	connect(horizontalSlider_R, SIGNAL(sliderMoved(int)), lcdNumber_R, SLOT(display(int)));
	connect(horizontalSlider_G, SIGNAL(sliderMoved(int)), lcdNumber_G, SLOT(display(int)));
	connect(horizontalSlider_B, SIGNAL(sliderMoved(int)), lcdNumber_B, SLOT(display(int)));
	connect(horizontalSlider_p, SIGNAL(sliderMoved(int)), lcdNumber_p, SLOT(display(int)));

	connect(horizontalSlider_R, SIGNAL(valueChanged(int)), this, SLOT(redSliderValueChanged(int)));
	connect(horizontalSlider_G, SIGNAL(valueChanged(int)), this, SLOT(greenSliderValueChanged(int)));
	connect(horizontalSlider_B, SIGNAL(valueChanged(int)), this, SLOT(blueSliderValueChanged(int)));
	connect(horizontalSlider_R, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(horizontalSlider_G, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(horizontalSlider_B, SIGNAL(sliderReleased()), this, SLOT(RGBsliderReleased()));
	connect(horizontalSlider_p, SIGNAL(valueChanged(int)), this, SLOT(pSliderValueChanged(int)));

}
void MainWindow::CreateMenus()
{
	menu_file_ = menuBar()->addMenu(tr("&File"));
	menu_file_->setStatusTip(tr("File menu"));
	menu_file_->addAction(action_new_);
	menu_file_->addAction(action_add_);
	menu_file_->addAction(action_open_);
	menu_file_->addAction(action_save_);
	menu_file_->addAction(action_saveas_);

	menu_edit_ = menuBar()->addMenu(tr("&Edit"));
	menu_edit_->setStatusTip(tr("Edit menu"));
	menu_edit_->addAction(action_downsample_);
	menu_edit_->addAction(action_random_color_);

	menu_help_ = menuBar()->addMenu(tr("&Help"));
	menu_help_->setStatusTip(tr("Help menu"));
	menu_help_->addAction(action_about_);
}
void MainWindow::CreateToolBars()
{
	toolbar_file_ = addToolBar(tr("File"));
	toolbar_file_->addAction(action_new_);
	toolbar_file_->addAction(action_open_);
	toolbar_file_->addAction(action_add_);
	toolbar_file_->addAction(action_save_);
	toolbar_file_->addAction(action_about_);

	// Add separator in toolbar 
	toolbar_file_->addSeparator();
	toolbar_file_->addAction(action_downsample_);
	toolbar_file_->addAction(action_random_color_);
}
void MainWindow::CreateStatusBar()
{
	statusBar()->showMessage(tr("Ready"));
}

void MainWindow::ActionAbout()
{
	QMessageBox::about(this, tr("About"), tr("PCL viewer was created by XCY"));
}


void MainWindow::Open()
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
	setA(255);
	//colorCloudDistances();
	viewer->updatePointCloud(cloud, "cloud");
	viewer->resetCamera();
	//update();
	qvtkWidget->update();

}

void MainWindow::setA(unsigned int a)
{
	for (int i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].r = a;
		cloud->points[i].g = a;
		cloud->points[i].b = a;
	}
}

void MainWindow::RandomColor()
{
	for (size_t i = 0; i < cloud->size(); i++) 
	{
		cloud->points[i].r = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
		cloud->points[i].g = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
		cloud->points[i].b = 255 * (1024 * rand() / (RAND_MAX + 1.0f));
	}
	viewer->updatePointCloud(cloud, "cloud");
	qvtkWidget->update();
}

void MainWindow::redSliderValueChanged(int value)
{
	red = value;
	printf("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void MainWindow::greenSliderValueChanged(int value)
{
	green = value;
	printf("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void MainWindow::blueSliderValueChanged(int value)
{
	blue = value;
	printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void MainWindow::pSliderValueChanged(int value)
{
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
	qvtkWidget->update();
}

void MainWindow::RGBsliderReleased()
{
	// Set the new color
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].r = red;
		cloud->points[i].g = green;
		cloud->points[i].b = blue;
	}
	viewer->updatePointCloud(cloud, "cloud");
	qvtkWidget->update();
}

void MainWindow::Add()
{
	QStringList filenames = QFileDialog::getOpenFileNames(this, tr("Open point cloud file"), QString::fromLocal8Bit(mycloud.dirname.c_str()), tr("Point cloud data(*.pcd *.ply);;All file(*.*)"));
	if (filenames.isEmpty())
		return;
	for (int i = 0; i != filenames.size(); i++) {
		// time start
		//timeStart();
		mycloud.cloud.reset(new PointCloudT);
		QString filename = filenames[i];
		std::string file_name = filename.toStdString();
		std::string subname = getFileName(file_name);

		// 更新状态栏
		//ui.statusBar->showMessage(QString::fromLocal8Bit(subname.c_str()) + ": " + QString::number(i) + "/" + QString::number(filenames.size()) + " point cloud loading...");

		int status = -1;
		if (filename.endsWith(".pcd", Qt::CaseInsensitive))
		{
			status = pcl::io::loadPCDFile(file_name, *(mycloud.cloud));
			if (mycloud.cloud->points[0].r == 0 && mycloud.cloud->points[0].g == 0 && mycloud.cloud->points[0].b == 0)
			{
				setA(255);
			}
		}
		else if (filename.endsWith(".ply", Qt::CaseInsensitive))
		{
			status = pcl::io::loadPLYFile(file_name, *(mycloud.cloud));
			if (mycloud.cloud->points[0].r == 0 && mycloud.cloud->points[0].g == 0 && mycloud.cloud->points[0].b == 0)
			{
				setA(255);
			}
		}

		else
		{
			//提示：无法读取除了.ply .pcd以外的文件
			QMessageBox::information(this, tr("File format error"), tr("Can't open files except .ply .pcd .obj"));
			return;
		}
		//提示：后缀没问题，但文件内容无法读取
		if (status != 0)
		{
			QMessageBox::critical(this, tr("Reading file error"), tr("We can not open the file"));
			return;
		}
		setA(255);  //设置点云为不透明
		mycloud.filename = file_name;
		mycloud.subname = subname;
		mycloud.dirname = file_name.substr(0, file_name.size() - subname.size());
		mycloud_vec.push_back(mycloud);  //将点云导入点云容器

										 // time of
		//time_cost = timeOff();
		//输出窗口
		//consoleLog("Add", QString::fromLocal8Bit(mycloud.subname.c_str()), QString::fromLocal8Bit(mycloud.filename.c_str()), "Time cost: " + time_cost + " s, Points: " + QString::number(mycloud.cloud->points.size()));

		//设置资源管理器
		//QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(subname.c_str()));
		//cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
		//ui.dataTree->addTopLevelItem(cloudName);

		//setWindowTitle("CloudViewer");
		total_points += mycloud.cloud->points.size();
	}
	ui.statusBar->showMessage("");
	showPointcloudAdd();
}

void MainWindow::showPointcloudAdd()
{
	for (int i = 0; i != mycloud_vec.size(); i++)
	{
		viewer->addPointCloud(mycloud_vec[i].cloud, "cloud" + QString::number(i).toStdString());
		viewer->updatePointCloud(mycloud_vec[i].cloud, "cloud" + QString::number(i).toStdString());
	}
	viewer->resetCamera();
	qvtkWidget->update();
}