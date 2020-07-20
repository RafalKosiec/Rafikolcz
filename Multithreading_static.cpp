#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <mutex>
#include <time.h>
#include <cstdlib>
//oddzielam includy pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/pcl_base.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/filters/boost.h>

#include "taskflow/taskflow.hpp"

typedef pcl::PointXYZ PointType;

std::mutex mutex; //THREAD
boost::mutex bMutex; //BOOST::THREAD
std::ofstream zapis("TestowyFold/dane.txt");
std::atomic<int> ile = 0;

void pass_filter(float min, float max, pcl::PointCloud<PointType>::Ptr cloud) {
	if (cloud->empty()) {
		std::cout << "pass_filter: Chmura do liczenia jest pusta!" << std::endl;
	}
	else {
		pcl::PassThrough<PointType> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(min, max);
		pass.filter(*cloud);
	}
}

//----------SAMODZIELNY PASS_FILTER <<DZIALA>>-------------------------
void pass_filter_samodzielny(float min, float max, pcl::PointCloud<PointType>::Ptr cloud) {
	if (cloud->empty()) {
		std::cout << "pass_filter_samodzielny: Chmura do liczenia jest pusta!" << std::endl;
	}
	else {
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud<PointType>(*cloud, indices); //to jest pierwszy filtr, wiec to tu zostawie; pass filter na poczatku usuwa NaNy
		indices.clear();
		for (int i = 0; i < cloud->size(); i++) {
			if (cloud->points[i].z < max && cloud->points[i].z > min) {
				indices.push_back(i);
			}
		}
		pcl::copyPointCloud(*cloud, indices, *cloud);
	}
}

void outlier_removal(int l_punktow, double odchylka_od_dewiacji, pcl::PointCloud<PointType>::Ptr cloud) {
	if (cloud->empty()) {
		std::cout << "oulier_removal: Chmura do liczenia jest pusta!" << std::endl;
	}
	else {
		pcl::StatisticalOutlierRemoval<PointType> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(l_punktow);
		sor.setStddevMulThresh(odchylka_od_dewiacji);
		sor.filter(*cloud);
	}
}

//----------SAMODZIELNY OUTLIER REMOVAL <<DZIALA>>-------------------------
void outlier_removal_samodzielny(int l_punktow, double odchylka_od_dewiacji, pcl::PointCloud<PointType>::Ptr cloud) {
	if (cloud->empty()) {
		std::cout << "oulier_removal_samodzielny: Chmura do liczenia jest pusta!" << std::endl;
	}
	else {
		std::vector<int> indices;

		pcl::search::Search<PointType>::Ptr szukacz;
		if (cloud->isOrganized()) szukacz.reset(new pcl::search::OrganizedNeighbor<PointType>());
		else szukacz.reset(new pcl::search::KdTree<PointType>(false));
		szukacz->setInputCloud(cloud);

		std::vector<int> nn_indices(l_punktow);
		std::vector<float> nn_dists(l_punktow);
		std::vector<float> distances(cloud->size());

		int valid_distances = 0;
		for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
			if (szukacz->nearestKSearch(cloud->points[i], l_punktow + 1, nn_indices, nn_dists) == 0)
			{
				distances[i] = 0.0;
				std::cout << "cos poszlonie tak //tu mial byc ten warning" << std::endl;
				continue;
			}

			double dist_sum = 0.0;
			for (int k = 1; k < l_punktow + 1; ++k)  // k = 0 is the query point
				dist_sum += sqrt(nn_dists[k]);
			distances[i] = static_cast<float> (dist_sum / l_punktow);
			valid_distances++;
		}

		double sum = 0, sq_sum = 0;
		for (size_t i = 0; i < distances.size(); ++i){
			sum += distances[i];
			sq_sum += distances[i] * distances[i];
		}

		double mean = sum / static_cast<double>(valid_distances);
		double variance = (sq_sum - sum * sum / static_cast<double>(valid_distances)) / (static_cast<double>(valid_distances) - 1);
		double stddev = sqrt(variance);

		double distance_threshold = mean + odchylka_od_dewiacji * stddev;

		for (int i = 0; i < static_cast<int>(cloud->size()); i++) {
			if (distances[i] < distance_threshold) {
				indices.push_back(i);
			}
		}
		pcl::copyPointCloud(*cloud, indices, *cloud);
	}
		
}

void downsampling_vox_grid(float wielkosc_liscia, pcl::PointCloud<PointType>::Ptr cloud) {
	if (cloud->empty()) {
		std::cout << "downsampling_vox_grid: Chmura do liczenia jest pusta!" << std::endl;
	}
	else {
		pcl::VoxelGrid<PointType> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(wielkosc_liscia, wielkosc_liscia, wielkosc_liscia);
		sor.filter(*cloud);
	}
}

void downsampling_vox_grid_samodzielny(float wielkosc_liscia, pcl::PointCloud<PointType>::Ptr cloud) {
	if (cloud->empty()) {
		std::cout << "downsampling_vox_grid_samodzielny: Chmura do liczenia jest pusta!" << std::endl;
	}
	else {

	}
}

void normals(double promien, pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
	if (cloud->empty()) {
		std::cout << "normals: Chmura normalnych albo zwykla jest pusta!" << std::endl;
	}
	else {
		pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
		pcl::NormalEstimation<PointType, pcl::Normal> ne;
		ne.setInputCloud(cloud);
		ne.setRadiusSearch(promien);
		//ne.setKSearch(25);
		ne.setSearchMethod(tree);
		ne.compute(*normals);
	}
}


void segmentationV2(pcl::PointCloud<PointType>::Ptr cloud) {
	if (cloud->empty()) {
		std::cout << "SegmentationV2: Chmura jest pusta!";
	}
	else {
		pcl::PointCloud<PointType>::Ptr cloud_f(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr cloud_plane(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr cloud_projected(new pcl::PointCloud<PointType>);
		pcl::SACSegmentation<PointType> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PCDWriter writer;
		pcl::PCDWriter writer1;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.02);
		int i = 0, nr_points = (int)cloud->points.size();
		while (cloud->points.size() > 0.3 * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud(cloud);
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<PointType> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			extract.setNegative(false);

			// Get the points associated with the planar surface
			extract.filter(*cloud_plane);

			// Remove the planar inliers, extract the rest
			extract.setNegative(true);
			extract.filter(*cloud_f);
			*cloud = *cloud_f;
		}
		pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
		tree->setInputCloud(cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointType> ec;
		ec.setClusterTolerance(0.015); // 2cm
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);

		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
				cloud_cluster->points.push_back(cloud->points[*pit]); //*
			}
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::cout << "PointCloud representing the Cluster" << j << ": " << cloud_cluster->points.size() << " data points." << std::endl;
			//std::cout << ile_tego_bylo;
			std::stringstream ss;
			ss << "cloud_cluster_" << j << ".pcd";
			if (j == 0) {
				std::stringstream nazwa;
				//nazwa << "Puszka_Coli_Wzor_nr" << ile_tego_bylo +40<< ".pcd";
				//writer.write<PointType>(nazwa.str(), *cloud_cluster, false);
			}
			writer.write<PointType>(ss.str(), *cloud_cluster, false); //*
			j++;
		}
		//ile_tego_bylo++;
		//pcl::PointCloud <PointType>::Ptr colored_cloud = ec.
		//cloud->swap(*colored_cloud); //JAK SIE ZMIENI NA RGBA TO TRZEBA 
	}
}

void pelnaFiltracja(int ile_iteracji=0) {

		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	

	if (ile_iteracji == 0) { //Przypadek dla openMP
		pcl::io::loadPCDFile("model.pcd", *cloud);

		std::vector<int> indices;
		pcl::removeNaNFromPointCloud<PointType>(*cloud, indices);
		pass_filter(-0.05, 0.05, cloud);
		outlier_removal(25, 2.0, cloud);
		downsampling_vox_grid(0.005f, cloud);
		normals(0.04, cloud, cloud_normals);

		//zapis <<ile<<": "<< cloud->size() << std::endl;
		//segmentationV2(cloud);
		ile++;
	}
	else { //Przypadek dla Thread i Boost
		for (int i = 0; i < ile_iteracji; i++) {
			pcl::io::loadPCDFile("model.pcd", *cloud);

			std::vector<int> indices;
			pcl::removeNaNFromPointCloud<PointType>(*cloud, indices);
			pass_filter(-0.05, 0.05, cloud);
			outlier_removal(25, 2.0, cloud);
			downsampling_vox_grid(0.005f, cloud);
			normals(0.04, cloud, cloud_normals);
			//zapis << ile << ": "<< cloud->size() << std::endl;

			//segmentationV2(cloud);
			ile++;
		}
	}
}

void pelnaFiltracjaTaskFlow(int liczba_threadow, int ile_iteracji) {

	tf::Executor executor;
	tf::Taskflow taskflow;
	int spread = ile_iteracji / liczba_threadow;
	for (int i = 0; i < liczba_threadow; i++) taskflow.emplace([=]() { pelnaFiltracja(spread); });

	executor.run(taskflow).wait();

}


void pelnaFiltarcjaBoost(int liczba_threadow, int ile_iteracji) {
	boost::thread_group tgroup;
	int spread = ile_iteracji / liczba_threadow;
	for (int i = 0; i < liczba_threadow; i++) {
		tgroup.create_thread(boost::bind(&pelnaFiltracja, spread));
		std::cout << tgroup.size() << std::endl;
	}
	tgroup.join_all();
}

void pelnaFiltracjaThreads(int liczba_threadow, int ile_iteracji) {
	std::vector<std::thread> threadVect;
	int spread = ile_iteracji / liczba_threadow;
	for (int i = 0; i < liczba_threadow; i++) {
		threadVect.emplace_back(pelnaFiltracja, spread);
		std::cout << threadVect.size() << std::endl;
	}
	for (auto& t : threadVect) {
		t.join();
	}
}

void pelnaFiltracjaOMP(int liczba_threadow, int ile_iteracji) {
	omp_set_num_threads(liczba_threadow); //Ile threadow ma używać OMP
	int i;
#pragma omp parallel for private(i) //OMP
	for (i = 0; i < ile_iteracji; i++) {
		pelnaFiltracja();
	}
}

int main()
{	
	//pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	//pcl::PointCloud<PointType>::Ptr cloud2(new pcl::PointCloud<PointType>);
	//pcl::io::loadPCDFile("model.pcd", *cloud);
	//pcl::io::loadPCDFile("model.pcd", *cloud2);
	//std::vector<int> isd;
	//std::vector<int> isd2;
	//pcl::removeNaNFromPointCloud(*cloud, isd);
	//pcl::removeNaNFromPointCloud(*cloud2, isd2);

	//std::cout << "c1: Przed: " << cloud->size() << std::endl;
	////pass_filter(-0.05, 0.05, cloud);
	//outlier_removal(25, 1.0, cloud);
	//std::cout << "c1: Po: " << cloud->size() << std::endl;

	//
	//std::cout << "c2: Przed: " << cloud2->size() << std::endl;
	////pass_filter_samodzielny(-0.05, 0.05, cloud2);
	//outlier_removal_samodzielny(25, 1.0, cloud2);
	//std::cout << "c2: Po: " << cloud2->size() << std::endl;

	time_t start = clock();

	//-------SPRAWDZANIE CZY FILTRY DZIALAJA JAK NALEZY
	/*for (int i = 0; i < cloud->size(); i++) {
		if (cloud->points[i].x == cloud2->points[i].x && cloud->points[i].y == cloud2->points[i].y && cloud->points[i].y == cloud2->points[i].y);
		else std::cout << "onie!" << std::endl;
	}*/

	pelnaFiltracjaOMP(4, 104);
	//pelnaFiltarcjaBoost(8, 104);
	//pelnaFiltracjaThreads(8, 104);
	//pelnaFiltracjaTaskFlow(8, 104);
	//pelnaFiltracja(104);

	time_t stop = clock();
	std::cout << "Czas wykonywania: " << (double)(stop - start) / CLOCKS_PER_SEC << std::endl;
	//std::cout<<"Po: " << cloud->size() << std::endl;

	//pcl::io::savePCDFile("model2.pcd", *cloud);
	//std::cout <<"Sprawdzenie poprawnosci: "<< ile<< std::endl;

	return(0);
}
