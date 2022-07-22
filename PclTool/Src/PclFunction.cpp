#include "PclFunction.h"

PointCloudT::Ptr PclFilterVoxel(PointCloudT::Ptr cloud_in, float leaf_size)
{
	pcl::VoxelGrid<PointT> voxel_grid;
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	voxel_grid.setInputCloud(cloud_in);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	voxel_grid.filter(*cloud_out);

	return cloud_out;
}

PointCloudT::Ptr PclFilterAvoxel(PointCloudT::Ptr cloud_in, float leaf_size)
{
	pcl::ApproximateVoxelGrid<PointT> avoxel_grid;
	avoxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
	avoxel_grid.setInputCloud(cloud_in);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	avoxel_grid.filter(*cloud_out);
	return cloud_out;
}


PointCloudT::Ptr PclFilterUniform(PointCloudT::Ptr cloud_in, float leaf_size)
{
	pcl::UniformSampling<PointT> uniform_samp;
	uniform_samp.setRadiusSearch(leaf_size);
	uniform_samp.setInputCloud(cloud_in);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	uniform_samp.filter(*cloud_out);

	return cloud_out;
}

PointCloudT::Ptr PclFilterRandom(PointCloudT::Ptr cloud_in, int num)
{
	pcl::RandomSample<PointT> random_sp;
	random_sp.setInputCloud(cloud_in);
	random_sp.setSample(num);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	random_sp.filter(*cloud_out);

	return cloud_out;
}

PointCloudT::Ptr PclFilterGridmin(PointCloudT::Ptr cloud_in, float resolutin)
{
	pcl::GridMinimum<PointT> gm(resolutin);
	gm.setInputCloud(cloud_in);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	gm.filter(*cloud_out);
	return cloud_out;
}

PointCloudT::Ptr PclFilterStatistical(PointCloudT::Ptr cloud_in, double stddev_mult, int nr_k)
{
	pcl::StatisticalOutlierRemoval<PointT> statical;
	statical.setInputCloud(cloud_in);
	statical.setMeanK(nr_k);
	statical.setStddevMulThresh(stddev_mult);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	statical.filter(*cloud_out);

	return cloud_out;
}

PointCloudT::Ptr PclFilterRadius(PointCloudT::Ptr cloud_in, double r_, int min_pts)
{
	pcl::RadiusOutlierRemoval<PointT> radius;
	radius.setInputCloud(cloud_in);
	radius.setRadiusSearch(r_);
	radius.setMinNeighborsInRadius(min_pts);
	PointCloudT::Ptr cloud_out(new PointCloudT);
	radius.filter(*cloud_out);

	return cloud_out;
}


void getScreenPos(double* displayPos, double* world_point, pcl::visualization::PCLVisualizer::Ptr viewer)
{
	vtkSmartPointer<vtkRenderer> renderer(viewer->getRendererCollection()->GetFirstRenderer());
	pcl::visualization::Camera camera_;
	viewer->getCameraParameters(camera_);

	double tmp_P[4], eventPos[4];
	/**@brief Use the window pos for worldPoint
	* very disgusting
	*/
	renderer->SetWorldPoint(camera_.window_pos);
	renderer->WorldToDisplay();
	renderer->GetDisplayPoint(tmp_P);

	tmp_P[0] = displayPos[0];
	tmp_P[1] = displayPos[1];

	renderer->SetDisplayPoint(tmp_P);
	renderer->DisplayToWorld();

	renderer->GetWorldPoint(eventPos);

	for (int i = 0; i < 3; ++i) {
		world_point[i] = eventPos[i];
	}
}

int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y)
{
	int i, j;
	j = poly_sides - 1;
	int res = 0;

	//对每一条边进行遍历，该边的两个端点，有一个必须在待检测点(x,y)的左边，且两个点中，有一个点的y左边比p.y小，另一个点的y比p.y大。
	for (i = 0; i < poly_sides; i++) {
		if (((poly_Y[i] < y && poly_Y[j] >= y) || (poly_Y[j] < y && poly_Y[i] >= y)) && (poly_X[i] <= x || poly_X[j] <= x))
		{   //用水平的直线与该边相交，求交点的x坐标。
			res ^= ((poly_X[i] + (y - poly_Y[i]) / (poly_Y[j] - poly_Y[i]) * (poly_X[j] - poly_X[i])) < x);
		}
		j = i;
	}
	return res;
}

PointCloudT::Ptr ProjectInliers(PointCloudT::Ptr cloud_in, PointCloudT::Ptr clicked_points_3d, pcl::visualization::PCLVisualizer::Ptr viewer, void* args)
{
	pcl::visualization::Camera camera_;
	viewer->getCameraParameters(camera_);


	/*焦点foacl 到 当前点pos 的向量*/
	PointT eyeLine_tmp = PointT(camera_.focal[0] - camera_.pos[0],
		camera_.focal[1] - camera_.pos[1], camera_.focal[2] - camera_.pos[2]);
	/*求模*/
	float mochang = sqrt(pow(eyeLine_tmp.x, 2) + pow(eyeLine_tmp.y, 2) + pow(eyeLine_tmp.z, 2));
	/*法向量*/
	PointT eyeLine = PointT(eyeLine_tmp.x / mochang, eyeLine_tmp.y / mochang, eyeLine_tmp.z / mochang);
	/* @brief :创建一个垂直于法向量过原点的平面
	*	(a-0)x + (b-0)y + (c-0)z = 0;
	*/
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = eyeLine.x;
	coefficients->values[1] = eyeLine.y;
	coefficients->values[2] = eyeLine.z;
	coefficients->values[3] = 0;

	PointCloudT::Ptr cloudCiecle_result(new PointCloudT); //投影范围
	PointCloudT::Ptr cloudIn_Prj(new PointCloudT);  //保存最后的结果

	pcl::ProjectInliers<PointT> proj;
	proj.setModelType(pcl::SACMODEL_PLANE); //投影类型
	proj.setInputCloud(clicked_points_3d);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloudCiecle_result);

	pcl::ProjectInliers<PointT> projCloudIn;
	projCloudIn.setModelType(pcl::SACMODEL_PLANE);
	projCloudIn.setInputCloud(cloud_in);
	projCloudIn.setModelCoefficients(coefficients);
	projCloudIn.filter(*cloudIn_Prj);

	int ret = -1;
	double* PloyXarr = new double[cloudCiecle_result->points.size()];
	double* PloyYarr = new double[cloudCiecle_result->points.size()];
	for (int i = 0; i < cloudCiecle_result->points.size(); i++)
	{
		PloyXarr[i] = cloudCiecle_result->points[i].x;
		PloyYarr[i] = cloudCiecle_result->points[i].y;
	}

	PointCloudT::Ptr cloud_out(new PointCloudT);
	for (int i = 0; i < cloudIn_Prj->points.size(); i++)
	{
		ret = inOrNot1(clicked_points_3d->points.size(), PloyXarr, PloyYarr, cloudIn_Prj->points[i].x, cloudIn_Prj->points[i].y);
		if (1 == ret)//表示在里面
		{
			cloud_out->points.push_back(cloud_in->points[i]);
		}//表示在外面
	}
	if (cloud_out->size() > 0)
		return cloud_out;
	else {
		return nullptr;
	}
}

pcl::PolygonMesh::Ptr PclGreedyTriangulation(const pcl::PointCloud<PointT>::Ptr cloud_in)
{
	/*法线估计对象*/
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation_object;
	/*存储估计的法线*/
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	/*定义kd树指针*/
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

	tree->setInputCloud(cloud_in);	//用cloud构建tree
	normal_estimation_object.setInputCloud(cloud_in);  //为法线估计对象设置输入点云
	normal_estimation_object.setSearchMethod(tree);	   //设置搜索方法	
	normal_estimation_object.setKSearch(20);	//设置k搜索k值
	normal_estimation_object.compute(*normals);	//估计法线结果保存到normals中

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals); //保存有向点云

	/*三角化相关定义*/
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZINormal>);
	tree2->setInputCloud(cloud_with_normals);
	pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3;
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

	gp3.setSearchRadius(1.0);
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 2);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);
	gp3.setInputCloud(cloud_with_normals);

	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*triangles);

	return triangles;
}

inline void signelVloume(PointT p1, PointT p2, PointT p3, float reference_plane_h, aream& areaMessage)
{

	float S_ = abs(p1.x * p2.y + p1.y * p3.x + p2.x * p3.y
		- p2.y * p3.x - p3.y * p1.x - p1.y * p2.x) / 2;

	float h_ = (p1.z + p2.z + p3.z) / 3 - reference_plane_h;

	areaMessage.S_ += S_;
	areaMessage.h_ += h_;
	areaMessage.volume_ += (S_ * h_);
	return;
}

void volumeOfMesh(pcl::PolygonMesh mesh, float reference_plane_h, aream& areaMessage)
{
	int i;
	PointCloudT::Ptr cloud(new PointCloudT);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	for (i = 0; i < mesh.polygons.size(); ++i) {
		PointT pt1 = cloud->points[mesh.polygons[i].vertices[0]];
		PointT pt2 = cloud->points[mesh.polygons[i].vertices[1]];
		PointT pt3 = cloud->points[mesh.polygons[i].vertices[2]];
		signelVloume(pt1, pt2, pt3, reference_plane_h, areaMessage);
	}

	areaMessage.h_ = areaMessage.h_ / i;
	return;
}
