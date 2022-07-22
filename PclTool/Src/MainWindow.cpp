#include "MainWindow.h"
#include <QMenuBar>
#include <Qmenu>
#include <QtoolBar>
#include <QStatusBar>

#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent),
	ui(new Ui::MainWindowClass)
{
	ui->setupUi(this);
	this->setWindowTitle("PclTool");
	this->setWindowIcon(QIcon(":/Res/Res/Mario.ico"));

	// Setup the cloud pointer
	cloud_.reset(new PointCloudT);
	// Setup the backup data
	cloud_original.reset(new PointCloudT);


	cloud_reference.reset(new PointCloudT);
	cloud_area.reset(new PointCloudT);

	clicked_points_3d.reset(new PointCloudT);
	cloud_volume.reset(new PointCloudT);

	triangles.reset(new pcl::PolygonMesh);

	cloud_name.push_back("cloud_");
	cloud_name.push_back("cloud_area");
	cloud_name.push_back("cloud_reference");
	cloud_name.push_back("cloud_volume");

	initView();

	/** @brief Create Menu */
	QMenuBar* menu_bar = new QMenuBar(this);
	this->setMenuBar(menu_bar);
	menu_bar->setStyleSheet("font-size : 16px");

	QMenu* file_menu = new QMenu("文件", menu_bar);
	QAction* open_action = new QAction(QIcon(":/Res/Res/OpenFile.png"), "打开文件");
	QAction* save_action = new QAction(QIcon(":/Res/Res/SaveFile.png"), "保存文件");
	QAction* backup_action = new QAction(QIcon(":/Res/Res/BackUp.png"), "恢复文件");
	QAction* quit_action = new QAction(QIcon(":/Res/Res/Exit.png"), "退出");
	file_menu->addAction(open_action);
	file_menu->addAction(save_action);
	file_menu->addAction(backup_action);
	file_menu->addSeparator();
	file_menu->addAction(quit_action);
	menu_bar->addMenu(file_menu);

	QMenu* view_menu = new QMenu("显示", menu_bar);
	QAction* view_background = new QAction(QIcon(":/Res/Res/BackGround.png"), "背景设置");
	QAction* view_render = new QAction(QIcon(":/Res/Res/CloudRender.png"), "点云渲染");
	QAction* view_color = new QAction(QIcon(":/Res/Res/PointColor.png"), "点云颜色");
	QAction* view_size = new QAction(QIcon(":/Res/Res/PointSize.png"), "点的大小");
	view_menu->addAction(view_background);
	view_menu->addAction(view_render);
	view_menu->addAction(view_color);
	view_menu->addAction(view_size);
	menu_bar->addMenu(view_menu);

	QMenu* filter_menu = new QMenu("滤波", menu_bar);

	QMenu* filter_sampling_menu = new QMenu("采样");
	QAction* filter_voxel = new QAction(QIcon(":/Res/Res/BackGround.png"), "体素滤波");
	QAction* filter_avoxel = new QAction(QIcon(":/Res/Res/BackGround.png"), "近似滤波");
	QAction* filter_uniform = new QAction(QIcon(":/Res/Res/BackGround.png"), "均匀采样");
	QAction* filter_random = new QAction(QIcon(":/Res/Res/BackGround.png"), "固定采样");
	QAction* filter_gridmin = new QAction(QIcon(":/Res/Res/BackGround.png"), "Gridmin采样");
	filter_sampling_menu->addAction(filter_voxel);
	filter_sampling_menu->addAction(filter_avoxel);
	filter_sampling_menu->addAction(filter_uniform);
	filter_sampling_menu->addAction(filter_random);
	filter_sampling_menu->addAction(filter_gridmin);

	QMenu* filter_denoising_menu = new QMenu("去噪");
	QAction* filter_statistical = new QAction(QIcon(":/Res/Res/BackGround.png"), "统计滤波");
	QAction* filter_radius = new QAction(QIcon(":/Res/Res/BackGround.png"), "半径滤波");
	filter_denoising_menu->addAction(filter_statistical);
	filter_denoising_menu->addAction(filter_radius);

	filter_menu->addMenu(filter_sampling_menu);
	filter_menu->addSeparator();
	filter_menu->addMenu(filter_denoising_menu);

	menu_bar->addMenu(filter_menu);

	connect(open_action, SIGNAL(triggered()), this, SLOT(Open_clicked()));
	connect(save_action, SIGNAL(triggered()), this, SLOT(Save_clicked()));
	connect(backup_action, SIGNAL(triggered()), this, SLOT(BackUp_clicked()));
	connect(quit_action, SIGNAL(triggered()), this, SLOT(Quit_clicked()));

	connect(view_background, SIGNAL(triggered(bool)), this, SLOT(View_background_pressed()));
	connect(view_render, SIGNAL(triggered(bool)), this, SLOT(View_render_pressed()));
	connect(view_color, SIGNAL(triggered(bool)), this, SLOT(View_color_pressed()));
	connect(view_size, SIGNAL(triggered(bool)), this, SLOT(View_size_pressed()));

	connect(filter_voxel, SIGNAL(triggered(bool)), this, SLOT(Filter_voxel_pressed()));
	connect(filter_avoxel, SIGNAL(triggered(bool)), this, SLOT(Filter_avoxel_pressed()));
	connect(filter_uniform, SIGNAL(triggered(bool)), this, SLOT(Filter_uniform_pressed()));
	connect(filter_random, SIGNAL(triggered(bool)), this, SLOT(Filter_random_pressed()));
	connect(filter_gridmin, SIGNAL(triggered(bool)), this, SLOT(Filter_gridmin_pressed()));
	connect(filter_statistical, SIGNAL(triggered(bool)), this, SLOT(Filter_statistical_pressed()));
	connect(filter_radius, SIGNAL(triggered(bool)), this, SLOT(Filter_radius_pressed()));
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::initView()
{
#if VTK_MAJOR_VERSION > 8
	auto renderer = vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	viewer_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
	ui->qvtkWidget->setRenderWindow(viewer_->getRenderWindow());
	viewer_->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
#else
	viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
	viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
#endif

	viewer_->registerPointPickingCallback(&MainWindow::pp_callback, *this);
	viewer_->registerAreaPickingCallback(&MainWindow::areapp_callback, *this);
	viewer_->registerMouseCallback(&MainWindow::mouseEventOccurred, *this);
	viewer_->registerKeyboardCallback(&MainWindow::keyboardEventOccurred, *this);

	// Color the randomly generated cloud
	viewer_->setBackgroundColor(0.1, 0.1, 0.1);
	viewer_->addPointCloud<PointT>(cloud_, cloud_name[0]);
	viewer_->resetCamera();
	reference_plane_h = 0.0f;
	areaMessage.S_ = 0.0f;
	areaMessage.h_ = 0.0f;
	areaMessage.volume_ = 0.0f;
	mouse_currect_x = 0;
	mouse_currect_y = 0;
}

void MainWindow::refreshViewe()
{
#if VTK_MAJOR_VERSION > 8
	ui->qvtkWidget->renderWindow()->Render();
#else
	ui->qvtkWidget->update();
#endif
}

void MainWindow::showMessage()
{
#if 0
	QString message_;
	message_.append("点云数量: ");
	message_.append(QString::number(cloud_->points.size()));
	ui->textBro_1->setText(message_);
#endif

	ui->textBro_1->clear();
	int size_tmp = static_cast<int>(cloud_->size());
	QString PointSize = QString("%1").arg(size_tmp);
	ui->textBro_1->insertPlainText("点云数量：" + PointSize);
	ui->textBro_1->insertPlainText("\n当前X：" + QString::number(mouse_currect_x)
		+ "\n当前Y：" + QString::number(mouse_currect_y));
	ui->textBro_1->setFont(QFont("Arial", 10, QFont::Bold));

	char16_t square = 0xB2;
	char16_t cube = 0xB3;
	ui->textBro_2->clear();
	ui->textBro_2->insertPlainText("参考平面高度: " + QString::number(reference_plane_h, 'f', 2) + " m");
	ui->textBro_2->insertPlainText("\n区域占地面积: " + QString::number(areaMessage.S_, 'f', 2) + " m" + QString::fromUtf16(&square, 1));
	ui->textBro_2->insertPlainText("\n区域平均高度: " + QString::number(areaMessage.h_, 'f', 2) + " m");
	ui->textBro_2->insertPlainText("\n区域体积:   " + QString::number(areaMessage.volume_, 'f', 2) + " m" + QString::fromUtf16(&cube, 1));
	ui->textBro_2->setFont(QFont("Arial", 10, QFont::Bold));
}


void MainWindow::getCloudScope(PointCloudT cloud_in)
{
	pcl::getMinMax3D(cloud_in, pMin, pMax);
}

void MainWindow::pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	if (event.getPointIndex() == -1)
		return;
	PointT currect_point;
	event.getPoint(currect_point.x, currect_point.y, currect_point.z);
	cloud_reference->points.push_back(currect_point);

	float height_ = 0;
	for (size_t i = 0; i < cloud_reference->size(); ++i) {
		height_ += cloud_reference->at(i).z;
	}

	reference_plane_h = height_ / cloud_reference->size();

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_reference, 255, 0, 0);
	viewer_->removePointCloud(cloud_name[2]);
	viewer_->addPointCloud(cloud_reference, red, cloud_name[2]);
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, cloud_name[2]);
	refreshViewe();
}

void MainWindow::areapp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
	double height_;
	std::vector<int> indices;
	if (event.getPointsIndices(indices) == false)
		return;
	for (size_t i = 0; i < indices.size(); ++i) {
		cloud_area->points.push_back(cloud_->points.at(indices[i]));
	}

	for (size_t i = 0; i < cloud_area->size(); ++i) {
		height_ += cloud_area->at(i).z;
	}

	triangles = PclGreedyTriangulation(cloud_area);

	viewer_->removePolygonMesh("my");
	viewer_->addPolygonMesh(*triangles, "my");
	viewer_->setRepresentationToSurfaceForAllActors();
	//viewer_->setRepresentationToWireframeForAllActors();

	areaMessage.S_ = 0.0f;
	areaMessage.h_ = 0.0f;
	areaMessage.volume_ = 0.0f;

	volumeOfMesh(*triangles, reference_plane_h, areaMessage);
	showMessage();

#if 0
	pcl::visualization::PointCloudColorHandlerCustom<PointT> renderer_(cloud_area, 0, 0, 0);
	viewer_->removePointCloud(cloud_name[1]);
	viewer_->addPointCloud(cloud_area, renderer_, cloud_name[1]);
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name[1]);
#endif

	refreshViewe();
	cloud_area->clear();
}



void MainWindow::mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* args)
{
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{

		if (isPickingMode) {
			mouse_currect_x = event.getX();
			mouse_currect_y = event.getY();

			double world_point[3];
			double displayPos[2] = { (double)mouse_currect_x, (double)mouse_currect_y };
			getScreenPos(displayPos, world_point, viewer_);

			curP = PointT(world_point[0], world_point[1], world_point[2]);

			if (!isFirstPick) {
				isFirstPick = true;
			}
			else {
				char str1[512];
				sprintf(str1, "line#%03d", line_id++);//名字不能重复
				viewer_->addLine(lastP, curP, 255, 0, 0, str1);
				refreshViewe();
				showMessage();
			}
			lastP = curP;
			//切割点云添加
			clicked_points_3d->push_back(curP);
		}
	}
}


void MainWindow::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* args)
{
	if (event.getKeySym() == "a" && event.keyDown())
	{
		isPickingMode = !isPickingMode;
		if (isPickingMode) {
			std::cout << endl << "start draw" << endl;
			isFirstPick = false;
			line_id = 0;
			clicked_points_3d->clear();
		}
		else {
			viewer_->removeAllShapes();
			cloud_volume = ProjectInliers(cloud_, clicked_points_3d, viewer_, args);

			if (cloud_volume != nullptr) {
				triangles = PclGreedyTriangulation(cloud_volume);
				viewer_->removePolygonMesh("my");
				viewer_->addPolygonMesh(*triangles, "my");
				viewer_->setRepresentationToSurfaceForAllActors();
				//viewer_->setRepresentationToWireframeForAllActors();

				areaMessage.S_ = 0.0f;
				areaMessage.h_ = 0.0f;
				areaMessage.volume_ = 0.0f;


				volumeOfMesh(*triangles, reference_plane_h, areaMessage);
			}
			showMessage();
			refreshViewe();
		}
	}
}

/***********************************************************************
*  @Camera View
************************************************************************/
void MainWindow::btn_Up_clicked()
{
	getCloudScope(*cloud_);

	float  distance_XOY = 1.855 * ((pMax.x - pMin.x) > (pMax.y - pMin.y)
		? (pMax.x - pMin.x) : (pMax.y - pMin.y));

	float center_x = (pMax.x + pMin.x) / 2;
	float center_y = (pMax.y + pMin.y) / 2;
	float center_z = (pMax.z + pMin.z) / 2;

	viewer_->setCameraPosition(0, 0, distance_XOY,
		center_x, center_y, center_z,
		0, 1, 0);
	refreshViewe();
}
void MainWindow::btn_Down_clicked()
{
	getCloudScope(*cloud_);

	float  distance_XOY = 1.855 * ((pMax.x - pMin.x) > (pMax.y - pMin.y)
		? (pMax.x - pMin.x) : (pMax.y - pMin.y));

	float center_x = (pMax.x + pMin.x) / 2;
	float center_y = (pMax.y + pMin.y) / 2;
	float center_z = (pMax.z + pMin.z) / 2;

	viewer_->setCameraPosition(0, 0, -distance_XOY,
		center_x, center_y, center_z,
		0, 1, 0);
	refreshViewe();
}
void MainWindow::btn_Forward_clicked()
{
	getCloudScope(*cloud_);

	float  distance_XOZ = 1.855 * ((pMax.x - pMin.x) > (pMax.z - pMin.z)
		? (pMax.x - pMin.x) : (pMax.z - pMin.z));

	float center_x = (pMax.x + pMin.x) / 2;
	float center_y = (pMax.y + pMin.y) / 2;
	float center_z = (pMax.z + pMin.z) / 2;


	viewer_->setCameraPosition(0, -distance_XOZ, 0,
		center_x, center_y, center_z,
		0, 0, 1);
	refreshViewe();
}
void MainWindow::btn_Back_clicked()
{
	getCloudScope(*cloud_);

	float  distance_XOZ = 1.855 * ((pMax.x - pMin.x) > (pMax.z - pMin.z)
		? (pMax.x - pMin.x) : (pMax.z - pMin.z));

	float center_x = (pMax.x + pMin.x) / 2;
	float center_y = (pMax.y + pMin.y) / 2;
	float center_z = (pMax.z + pMin.z) / 2;

	viewer_->setCameraPosition(0, distance_XOZ, 0,
		center_x, center_y, center_z,
		0, 0, 1);
	refreshViewe();
}
void MainWindow::btn_Left_clicked()
{
	getCloudScope(*cloud_);

	float  distance_YOZ = 1.855 * ((pMax.y - pMin.y) > (pMax.z - pMin.z)
		? (pMax.y - pMin.y) : (pMax.z - pMin.z));

	float center_x = (pMax.x + pMin.x) / 2;
	float center_y = (pMax.y + pMin.y) / 2;
	float center_z = (pMax.z + pMin.z) / 2;

	viewer_->setCameraPosition(-distance_YOZ, 0, 0,
		center_x, center_y, center_z,
		0, 0, 1);
	refreshViewe();
}
void MainWindow::btn_Right_clicked()
{
	getCloudScope(*cloud_);

	float  distance_YOZ = 1.855 * ((pMax.y - pMin.y) > (pMax.z - pMin.z)
		? (pMax.y - pMin.y) : (pMax.z - pMin.z));

	float center_x = (pMax.x + pMin.x) / 2;
	float center_y = (pMax.y + pMin.y) / 2;
	float center_z = (pMax.z + pMin.z) / 2;

	viewer_->setCameraPosition(distance_YOZ, 0, 0,
		center_x, center_y, center_z,
		0, 0, 1);
	refreshViewe();
}

void MainWindow::btn_Filter_I_clicked()
{
	if (!cloud_->empty()) {
		PointCloudT::Ptr cloud_tmp(new PointCloudT);

		pcl::copyPointCloud(*cloud_, *cloud_tmp);
		pcl::visualization::PointCloudColorHandlerGenericField<PointT>render(
			cloud_tmp->makeShared(), "intensity");

		// If point cloud contains NaN values, remove them before updating the visualizer point cloud
		if (cloud_tmp->is_dense)
			pcl::copyPointCloud(*cloud_tmp, *cloud_);
		else {
			PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
			std::vector<int> vec;
			pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
		}

		viewer_->updatePointCloud(cloud_->makeShared(), render, cloud_name[0]);
		refreshViewe();
	}
}


/**********************************************************************
*	@Menu: File Menu
***********************************************************************/
void CreateCloudFormTxt(const std::string& file_path, PointCloudT::Ptr cloud)
{
	std::ifstream file(file_path.c_str());
	std::string line;
	PointT point;
	while (getline(file, line)) {
		std::stringstream ss(line);
		ss >> point.x;
		ss >> point.y;
		ss >> point.z;
		cloud->push_back(point);
	}
	file.close();
}


void MainWindow::Open_clicked()
{
	viewer_->close();
	cloud_->clear();
	clicked_points_3d->clear();
	cloud_reference->clear();
	cloud_original->clear();

	initView();

	QString fileName = QFileDialog::getOpenFileName(this, tr("open file"),
		"", tr("pcb files(*.pcd *.ply *.txt) ;;All files(*.*)"));

	PointCloudT::Ptr cloud_tmp(new PointCloudT);

	if (fileName.isEmpty())
		return;

	bool return_status = 0;
	if (fileName.endsWith("ply"))
		return_status = pcl::io::loadPLYFile(fileName.toStdString(), *cloud_tmp);
	else if (fileName.endsWith("pcd"))
		return_status = pcl::io::loadPCDFile(fileName.toStdString(), *cloud_tmp);
	else if (fileName.endsWith("txt"))
		CreateCloudFormTxt(fileName.toStdString(), cloud_tmp);
	else {
		QMessageBox::warning(this, "Warning", "点云读取格式错误！");
		return;
	}

	if (return_status != 0) {
		PCL_ERROR("Error reading point cloud %s\n", fileName.toStdString().c_str());
		return;
	}

	// If point cloud contains NaN values, remove them before updating the visualizer point cloud
	if (cloud_tmp->is_dense) {
		pcl::copyPointCloud(*cloud_tmp, *cloud_);
		/* to the backup data*/
		pcl::copyPointCloud(*cloud_tmp, *cloud_original);
	}
	else {
		PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
		std::vector<int> vec;
		pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
	}
	viewer_->addPointCloud<PointT>(cloud_volume, cloud_name[3]);
	viewer_->addPointCloud<PointT>(cloud_reference, cloud_name[2]);
	viewer_->addPointCloud<PointT>(cloud_area, cloud_name[1]);

	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);
	viewer_->resetCamera();

	refreshViewe();
	showMessage();
}

void MainWindow::Save_clicked()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Open point cloud"),
		"", tr("Point cloud data (*.pcd *.ply)"));
	if (cloud_->empty())
		return;
	else {
		if (fileName.isEmpty())
			return;
		int return_status;
		if (fileName.endsWith(".pcd", Qt::CaseInsensitive))
			return_status = pcl::io::savePCDFileBinary(fileName.toStdString(), *cloud_->makeShared());
		else if (fileName.endsWith(".ply", Qt::CaseInsensitive))
			return_status = pcl::io::savePLYFileBinary(fileName.toStdString(), *cloud_->makeShared());
		else {
			fileName.append(".ply");
			return_status = pcl::io::savePLYFileBinary(fileName.toStdString(), *cloud_->makeShared());
		}

		if (return_status != 0) {
			PCL_ERROR("Error writing point cloud %s\n", fileName.toStdString().c_str());
			return;
		}
	}
}

void MainWindow::BackUp_clicked()
{
	initView();
	cloud_area->clear();
	viewer_->removePointCloud(cloud_name[1]);
	viewer_->addPointCloud<PointT>(cloud_area, cloud_name[1]);

	cloud_reference->clear();
	viewer_->removePointCloud(cloud_name[2]);
	viewer_->addPointCloud<PointT>(cloud_reference, cloud_name[2]);

	pcl::copyPointCloud(*cloud_original, *cloud_);
	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);
	viewer_->resetCamera();
	refreshViewe();
	showMessage();
}

void MainWindow::Quit_clicked()
{
	this->close();
}

/********************************************************************************
*	@Menu: View Menu
*********************************************************************************/
void MainWindow::View_background_pressed()
{
	dialog_color = new ViewSelectColor();
	QColor color = dialog_color->getColor();
	background_color = color;
	viewer_->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
	refreshViewe();
	return;
}


void MainWindow::View_render_pressed()
{
	dialog_render = new ViewRendering();
	connect(dialog_render, SIGNAL(sendData(QString)), this,
		SLOT(Rendering_setting(QString)));
	if (dialog_render->exec() == QDialog::Accepted)
		delete dialog_render;
}

void MainWindow::Rendering_setting(QString data)
{
	if (!cloud_->empty()) {
		PointCloudT::Ptr cloud_tmp(new PointCloudT);

		pcl::copyPointCloud(*cloud_, *cloud_tmp);
		pcl::visualization::PointCloudColorHandlerGenericField<PointT>render(
			cloud_tmp->makeShared(), data.toStdString());

		// If point cloud contains NaN values, remove them before updating the visualizer point cloud
		if (cloud_tmp->is_dense)
			pcl::copyPointCloud(*cloud_tmp, *cloud_);
		else {
			PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
			std::vector<int> vec;
			pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
		}

		viewer_->updatePointCloud(cloud_->makeShared(), render, cloud_name[0]);
		refreshViewe();
	}
}


void MainWindow::View_color_pressed()
{
	dialog_color = new ViewSelectColor();
	QColor color = dialog_color->getColor();
	if (!cloud_->empty() && (color != background_color)) {

		PointCloudT::Ptr cloud_tmp(new PointCloudT);
		pcl::copyPointCloud(*cloud_, *cloud_tmp);

		pcl::visualization::PointCloudColorHandlerCustom<PointT>color_selected(
			cloud_tmp->makeShared(), color.redF() * 255, color.greenF() * 255, color.blueF() * 255);

		// If point cloud contains NaN values, remove them before updating the visualizer point cloud
		if (cloud_tmp->is_dense)
			pcl::copyPointCloud(*cloud_tmp, *cloud_);
		else {
			PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
			std::vector<int> vec;
			pcl::removeNaNFromPointCloud(*cloud_tmp, *cloud_, vec);
		}

		viewer_->updatePointCloud(cloud_->makeShared(), color_selected, cloud_name[0]);
		refreshViewe();
	}
	refreshViewe();
	return;
}


void MainWindow::View_size_pressed()
{
	dialog_size = new ViewPointSize();
	connect(dialog_size, SIGNAL(sendData(QString)), this, SLOT(PointSize_setting(QString)));
	if (dialog_size->exec() == QDialog::Accepted)
		delete dialog_size;
}

void MainWindow::PointSize_setting(QString data)
{
	point_size = data.toInt();
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_name[0]);
	refreshViewe();
}

/*************************************************************
*	@Menu: Filter Menu
**************************************************************/

void MainWindow::Filter_voxel_pressed()
{
	dialog_voxel = new FilterVoxel();
	connect(dialog_voxel, SIGNAL(sendData(QString)), this, SLOT(Filter_voxel_run(QString)));
	if (dialog_voxel->exec() == QDialog::Accepted)
		delete dialog_voxel;
}

void MainWindow::Filter_voxel_run(QString data)
{
	if (cloud_->empty()) {
		QMessageBox::warning(this, "Warning", "无点云输入!");
		return;
	}
	else {
		if (data.isEmpty()) {
			QMessageBox::warning(this, "Warning", "参数格式输入错误!");
			return;
		}
	}

	float size = data.toFloat();
	PointCloudT::Ptr cloud_out(new PointCloudT);
	cloud_out = PclFilterVoxel(cloud_, size);
	pcl::copyPointCloud(*cloud_out, *cloud_);
	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);

	refreshViewe();
	showMessage();
}

void MainWindow::Filter_avoxel_pressed()
{
	dialog_avoxel = new FilterAvoxel();
	connect(dialog_avoxel, SIGNAL(sendData(QString)), this, SLOT(Filter_avoxel_run(QString)));
	if (dialog_avoxel->exec() == QDialog::Accepted)
		delete dialog_voxel;
}

void MainWindow::Filter_avoxel_run(QString data)
{
	if (cloud_->empty()) {
		QMessageBox::warning(this, "Warning", "无点云输入!");
		return;
	}
	else {
		if (data.isEmpty()) {
			QMessageBox::warning(this, "Warning", "参数格式输入错误!");
			return;
		}
	}

	float size = data.toFloat();
	PointCloudT::Ptr cloud_out(new PointCloudT);
	cloud_out = PclFilterAvoxel(cloud_, size);
	pcl::copyPointCloud(*cloud_out, *cloud_);
	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);

	refreshViewe();
	showMessage();
}

void MainWindow::Filter_uniform_pressed()
{
	dialog_uniform = new FilterUniform();
	connect(dialog_uniform, SIGNAL(sendData(QString)), this, SLOT(Filter_uniform_run(QString)));
	if (dialog_uniform->exec() == QDialog::Accepted)
		delete dialog_uniform;
}

void MainWindow::Filter_uniform_run(QString data)
{
	if (cloud_->empty()) {
		QMessageBox::warning(this, "Warning", "无点云输入!");
		return;
	}
	else {
		if (data.isEmpty()) {
			QMessageBox::warning(this, "Warning", "参数格式输入错误!");
			return;
		}
	}

	float size = data.toFloat();
	PointCloudT::Ptr cloud_out(new PointCloudT);
	cloud_out = PclFilterUniform(cloud_, size);
	pcl::copyPointCloud(*cloud_out, *cloud_);
	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);

	refreshViewe();
	showMessage();
}

void MainWindow::Filter_random_pressed()
{
	dialog_random = new FilterRandom();
	connect(dialog_random, SIGNAL(sendData(QString)), this, SLOT(Filter_random_run(QString)));
	if (dialog_random->exec() == QDialog::Accepted)
		delete dialog_random;
}

void MainWindow::Filter_random_run(QString data)
{
	if (cloud_->empty()) {
		QMessageBox::warning(this, "Warning", "无点云输入!");
		return;
	}
	else {
		if (data.isEmpty()) {
			QMessageBox::warning(this, "Warning", "参数格式输入错误!");
			return;
		}
	}

	int num = data.toInt();
	PointCloudT::Ptr cloud_out(new PointCloudT);
	cloud_out = PclFilterRandom(cloud_, num);
	pcl::copyPointCloud(*cloud_out, *cloud_);
	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);

	refreshViewe();
	showMessage();
}


void MainWindow::Filter_gridmin_pressed()
{
	dialog_gridmin = new FilterGridmin();
	connect(dialog_gridmin, SIGNAL(sendData(QString)), this, SLOT(Filter_gridmin_run(QString)));
	if (dialog_gridmin->exec() == QDialog::Accepted)
		delete dialog_gridmin;
}
void MainWindow::Filter_gridmin_run(QString data)
{
	if (cloud_->empty()) {
		QMessageBox::warning(this, "Warning", "无点云输入!");
		return;
	}
	else {
		if (data.isEmpty()) {
			QMessageBox::warning(this, "Warning", "参数格式输入错误!");
			return;
		}
	}

	float size = data.toFloat();
	PointCloudT::Ptr cloud_out(new PointCloudT);
	cloud_out = PclFilterGridmin(cloud_, size);
	pcl::copyPointCloud(*cloud_out, *cloud_);
	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);

	refreshViewe();
	showMessage();
}

void MainWindow::Filter_statistical_pressed()
{
	dialog_statistical = new FilterStatistical();
	connect(dialog_statistical, SIGNAL(sendData(QString, QString)), this, SLOT(Filter_statistical_run(QString, QString)));
	if (dialog_statistical->exec() == QDialog::Accepted)
		delete dialog_statistical;
}
void MainWindow::Filter_statistical_run(QString data_1, QString data_2)
{
	if (cloud_->empty()) {
		QMessageBox::warning(this, "Warning", "无点云输入!");
		return;
	}
	else {
		if (data_1.isEmpty() || data_2.isEmpty()) {
			QMessageBox::warning(this, "Warning", "参数格式输入错误!");
			return;
		}
	}

	double stddev_mult = data_1.toDouble();
	int nr_k = data_2.toInt();

	PointCloudT::Ptr cloud_out(new PointCloudT);
	cloud_out = PclFilterStatistical(cloud_, stddev_mult, nr_k);
	pcl::copyPointCloud(*cloud_out, *cloud_);
	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);


	refreshViewe();
	showMessage();
}


void MainWindow::Filter_radius_pressed()
{
	dialog_radius = new FilterRadius();
	connect(dialog_radius, SIGNAL(sendData(QString, QString)), this, SLOT(Filter_radius_run(QString, QString)));
	if (dialog_radius->exec() == QDialog::Accepted)
		delete dialog_radius;
}

void MainWindow::Filter_radius_run(QString data_1, QString data_2)
{
	if (cloud_->empty()) {
		QMessageBox::warning(this, "Warning", "无点云输入!");
		return;
	}
	else {
		if (data_1.isEmpty() || data_2.isEmpty()) {
			QMessageBox::warning(this, "Warning", "参数格式输入错误!");
			return;
		}
	}
	double r_ = data_1.toDouble();
	int min_pts = data_2.toInt();

	PointCloudT::Ptr cloud_out(new PointCloudT);
	cloud_out = PclFilterRadius(cloud_, r_, min_pts);
	pcl::copyPointCloud(*cloud_out, *cloud_);
	viewer_->updatePointCloud<PointT>(cloud_, cloud_name[0]);

	refreshViewe();
	showMessage();
}

