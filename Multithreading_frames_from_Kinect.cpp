

//WIELOWATKOWOSC Z KINECTEM, WERSJA OKROJONA, DO WGLĄDU W KODZIE, ZOSTANIE JEDNA FUNKCJA FILTRUJĄCA, NAJPROSTSZY PRZYPADEK DWÓCH CHMUR PRZETWARZANYCH

#include <sstream>
#include <stdexcept>

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <wrl/client.h>
using namespace Microsoft::WRL;

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <mutex>
#include <time.h>
#include <cstdlib>
#include <vector>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/vfh.h>
#include <pcl/features/crh.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/project_inliers.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

#include "taskflow/taskflow.hpp"

typedef pcl::PointXYZRGBA PointType; //ZMIENIŁEM Z RGBA I JAKOŚ DZIAŁA

// Error Check
#define ERROR_CHECK( ret )                                        \
    if( FAILED( ret ) ){                                          \
        std::stringstream ss;                                     \
        ss << "failed " #ret " " << std::hex << ret << std::endl; \
        throw std::runtime_error( ss.str().c_str() );             \
    }

time_t start;
int ile_klatek = 0;

class Kinect
{
private:
	// Sensor
	ComPtr<IKinectSensor> kinect;

	// Coordinate Mapper
	ComPtr<ICoordinateMapper> coordinateMapper;

	// Reader
	ComPtr<IColorFrameReader> colorFrameReader;
	ComPtr<IDepthFrameReader> depthFrameReader;

	// Color Buffer
	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;

	// Depth Buffer
	std::vector<UINT16> depthBuffer;
	int depthWidth;
	int depthHeight;
	unsigned int depthBytesPerPixel;

	// PCL
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PointCloud<PointType>::Ptr cloud;

	//DODANE PRZEZE MNIE
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals; //Potrzebne do paru funkcji pcl

	boost::mutex mutex_update;
	boost::mutex mutex_przetwarzanie;
	boost::mutex mutex_wyswietlanie;
	pcl::PointCloud<PointType>::Ptr cloud2;
	pcl::PointCloud<PointType>::Ptr cloud3;
	pcl::PointCloud<PointType>::Ptr cloud4;
	pcl::PointCloud<PointType>::Ptr cloud5; 
	pcl::PointCloud<PointType>::Ptr cloud6; 
	size_t counter;
	std::vector<std::thread> thready;

	pcl::PointCloud<PointType>::Ptr cloud_out;

	int cnt1, cnt2, cnt3, cnt4;

public:
	// Constructor
	Kinect()
	{
		// Initialize
		initialize();
	}

	// Destructor
	~Kinect()
	{
		// Finalize
		finalize();
	}

	//funkcja robienie threadow
	void runThreads() {
		thready.push_back(std::thread(&Kinect::update, this)); //thread pobierajacy
		thready.push_back(std::thread(&Kinect::przetwarzaj, this, 1)); //thread przetwarzajacy
		thready.push_back(std::thread(&Kinect::przetwarzaj, this, 2)); //thread przetwarzajacy
		thready.push_back(std::thread(&Kinect::przetwarzaj, this, 3)); //thread przetwarzajacy
		thready.push_back(std::thread(&Kinect::przetwarzaj, this, 4)); //thread przetwarzajacy

		start = clock();

		for (auto& i : thready) {
			i.detach();
		}

		int i = 0;
		while (i < 10) {
			while (!viewer->wasStopped()) drawAndShow();
			i++;
		}

	}

private:

	//----------------O: RAZ PRZY KONSTRUKTORZE---------------------------

	// Initialize
	void initialize()
	{
		// Initialize Sensor
		initializeSensor();

		// Initialize Color
		initializeColor();

		// Initialize Depth
		initializeDepth();

		// Initialize Point Cloud
		initializePointCloud();

		counter = 0;
		cnt1 = 0;
		cnt2 = 0;
		cnt3 = 0;
		cnt4 = 0;
	}

	// Initialize Sensor
	inline void initializeSensor()
	{
		// Open Sensor
		ERROR_CHECK(GetDefaultKinectSensor(&kinect));

		ERROR_CHECK(kinect->Open());

		// Check Open
		BOOLEAN isOpen = FALSE;
		ERROR_CHECK(kinect->get_IsOpen(&isOpen));
		if (!isOpen) {
			throw std::runtime_error("failed IKinectSensor::get_IsOpen( &isOpen )");
		}

		// Retrieve Coordinate Mapper
		ERROR_CHECK(kinect->get_CoordinateMapper(&coordinateMapper));
	}

	// Initialize Color
	inline void initializeColor()
	{
		// Open Color Reader
		ComPtr<IColorFrameSource> colorFrameSource;
		ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
		ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

		// Retrieve Color Description
		ComPtr<IFrameDescription> colorFrameDescription;
		ERROR_CHECK(colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription));
		ERROR_CHECK(colorFrameDescription->get_Width(&colorWidth)); // 1920
		ERROR_CHECK(colorFrameDescription->get_Height(&colorHeight)); // 1080
		ERROR_CHECK(colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel)); // 4

		// Allocation Color Buffer
		colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
	}

	// Initialize Depth
	inline void initializeDepth()
	{
		// Open Depth Reader
		ComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
		ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

		// Retrieve Depth Description
		ComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK(depthFrameSource->get_FrameDescription(&depthFrameDescription));
		ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth)); // 512
		ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight)); // 424
		ERROR_CHECK(depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel)); // 2

		// Allocation Depth Buffer
		depthBuffer.resize(depthWidth * depthHeight);
	}

	// Initialize Point Cloud
	inline void initializePointCloud()
	{

		// Create Point Cloud
		cloud = boost::make_shared<pcl::PointCloud<PointType>>();
		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->points.resize(cloud->height * cloud->width);
		cloud->is_dense = false;

		//dodane przeze mnie!!!
		cloud2 = boost::make_shared<pcl::PointCloud<PointType>>();
		cloud2->width = static_cast<uint32_t>(depthWidth);
		cloud2->height = static_cast<uint32_t>(depthHeight);
		cloud2->points.resize(cloud2->height * cloud2->width);
		cloud2->is_dense = false;

		cloud3 = boost::make_shared<pcl::PointCloud<PointType>>();
		cloud3->width = static_cast<uint32_t>(depthWidth);
		cloud3->height = static_cast<uint32_t>(depthHeight);
		cloud3->points.resize(cloud3->height * cloud3->width);
		cloud3->is_dense = false;

		cloud4 = boost::make_shared<pcl::PointCloud<PointType>>();
		cloud4->width = static_cast<uint32_t>(depthWidth);
		cloud4->height = static_cast<uint32_t>(depthHeight);
		cloud4->points.resize(cloud4->height * cloud4->width);
		cloud4->is_dense = false;

		cloud5 = boost::make_shared<pcl::PointCloud<PointType>>();
		cloud5->width = static_cast<uint32_t>(depthWidth);
		cloud5->height = static_cast<uint32_t>(depthHeight);
		cloud5->points.resize(cloud5->height * cloud5->width);
		cloud5->is_dense = false;

		cloud6 = boost::make_shared<pcl::PointCloud<PointType>>();
		cloud6->width = static_cast<uint32_t>(depthWidth);
		cloud6->height = static_cast<uint32_t>(depthHeight);
		cloud6->points.resize(cloud6->height * cloud6->width);
		cloud6->is_dense = false;

		cloud_out = boost::make_shared<pcl::PointCloud<PointType>>();
		cloud_out->width = static_cast<uint32_t>(depthWidth);
		cloud_out->height = static_cast<uint32_t>(depthHeight);
		cloud_out->points.resize(cloud_out->height * cloud_out->width);
		cloud_out->is_dense = false;

		// Create PCLVisualizer
		viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");

		// Initialize camera position
		viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);

		// Add Coordinate System
		viewer->addCoordinateSystem(0.1);
	}

	//----------------C: RAZ PRZY KONSTRUKTORZE--------------------------


	//----------------O: RAZ PRZY DESTRUKTORZE---------------------------
	// Finalize
	void finalize()
	{
		// Close Sensor
		if (kinect != nullptr) {
			kinect->Close();
		}
	}
	//----------------C: RAZ PRZY DESTRUKTORZE---------------------------

		// Update Color
	inline void updateColor()
	{
		// Retrieve Color Frame
		ComPtr<IColorFrame> colorFrame;
		const HRESULT ret = colorFrameReader->AcquireLatestFrame(&colorFrame);
		if (FAILED(ret)) {
			return;
		}

		// Convert Format ( YUY2 -> BGRA )
		ERROR_CHECK(colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra));
	}

	// Update Depth
	inline void updateDepth()
	{
		// Retrieve Depth Frame
		ComPtr<IDepthFrame> depthFrame;
		const HRESULT ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (FAILED(ret)) {
			return;
		}

		// Retrieve Depth Data
		ERROR_CHECK(depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]));
	}

	// Update Point Cloud
	inline void updatePointCloud()
	{
		// Reset Point Cloud
		if (counter % 4 == 0) cloud->clear();
		else if (counter % 4 == 1) cloud2->clear();
		else if (counter % 4 == 2) cloud3->clear();
		else cloud4->clear();
		// Convert to Point Cloud
		for (int depthY = 0; depthY < depthHeight; depthY++) {
			for (int depthX = 0; depthX < depthWidth; depthX++) {
				PointType point; //tu zmieniłem

				// Retrieve Mapped Coordinates
				DepthSpacePoint depthSpacePoint = { static_cast<float>(depthX), static_cast<float>(depthY) };
				UINT16 depth = depthBuffer[depthY * depthWidth + depthX];
				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				ERROR_CHECK(coordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint));

				// Set Color to Point
				int colorX = static_cast<int>(colorSpacePoint.X + 0.5f);
				int colorY = static_cast<int>(colorSpacePoint.Y + 0.5f);
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					unsigned int colorIndex = (colorY * colorWidth + colorX) * colorBytesPerPixel;
					point.b = colorBuffer[colorIndex + 0];
					point.g = colorBuffer[colorIndex + 1];
					point.r = colorBuffer[colorIndex + 2];
					point.a = colorBuffer[colorIndex + 3];
				}

				// Retrieve Mapped Coordinates
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				ERROR_CHECK(coordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint));

				// Set Depth to Point
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
				}

				// Set Point to Point Cloud
				if (counter % 4 == 0) cloud->push_back(point);
				if (counter % 4 == 1) cloud2->push_back(point);
				if (counter % 4 == 2) cloud3->push_back(point);
				else cloud4->push_back(point);
			}
		}
	}
















	//----------------O: CYKLICZNIE---------------------------
	// Update Data
	void update()
	{
		while (!viewer->wasStopped()) {
			if (mutex_update.try_lock()) {

				// Update Color
				updateColor();

				// Update Depth
				updateDepth();

				// Update Point Cloud
				updatePointCloud();
				if (counter > 100000) counter = 0;
				counter++;

				cout << cloud->size() << " " << cloud2->size() << std::endl;
				mutex_update.unlock();
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	// Draw Data
	void draw()
	{
		// Draw Point Cloud
		drawPointCloud();
	}

	// Draw Point Cloud
	inline void drawPointCloud()
	{
			// Update Point Cloud
		if (!viewer->updatePointCloud(cloud_out, "1")) {
			viewer->addPointCloud(cloud_out, "1");
		}
	}

	// Show Data
	void show()
	{
			// Show Point Cloud
			showPointCloud();
	}

	// Show Point Cloud
	inline void showPointCloud()
	{
			// Update Viewer
			viewer->spinOnce();

	}

	void drawAndShow() {
		while (!viewer->wasStopped()) {
			if (mutex_wyswietlanie.try_lock()) {

				show();
				boost::mutex::scoped_try_lock lock(mutex_update);
				if (lock.owns_lock() && cloud_out) {
					draw();
				}
				mutex_wyswietlanie.unlock();
			}
		}
	}
	//----------------C: CYKLICZNIE---------------------------

	//----------------O: PRZETWARZANIE---------------------------

	//tu zmieniamy parametry filtrow
	void przetwarzaj(int id) {
		while(!viewer->wasStopped()){
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			if (mutex_update.try_lock()) {
				if (id == 1) cnt1++;
				if (id == 2) cnt2++;
				if (id == 3) cnt3++;
				if (id == 4) cnt4++;
				pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>);

				if (id==1) pcl::copyPointCloud(*cloud, *cloud_temp);
				else if(id==2) pcl::copyPointCloud(*cloud2, *cloud_temp);
				else if(id==3) pcl::copyPointCloud(*cloud3, *cloud_temp);
				else pcl::copyPointCloud(*cloud4, *cloud_temp);
				mutex_update.unlock();
				//filtr srodkowo przepustowy
				pass_filter(cloud_temp, 0.5, 2.0);

				//usuwanie punktow pojedynczych lub grup za daleko od pozostałych
				outlier_removal(cloud_temp, 25, 2.0);

				//downsampling metrodą wokseli
				downsampling_vox_grid(cloud_temp, 0.02f);

				//obliczenie normalnych
				//normals(0.04);

				//segmentacja rozrostem obszarów
				//segmentation();

				//segmentacja euclidean cluster
				//segmentationV2();
				if (cnt2 >= cnt1) {
					if (cnt3 >= cnt2) {
						if (cnt4 >= cnt3) {
							do {
								std::this_thread::sleep_for(std::chrono::milliseconds(2));
							} while (cnt2<cnt1 && cnt3<cnt2 && cnt4<cnt3);
						}
					}
				}


				mutex_wyswietlanie.lock();
				ile_klatek++;
				pcl::copyPointCloud(*cloud_temp, *cloud_out);
				mutex_wyswietlanie.unlock();
				//cout << "przetwarzanie " << cloud->size() << " "<<cloud2->size() << endl;
			}
		}
	}


	//FILTR ŚRODKOWOPRZEPUSTOWY --min--max-- odleglosci w metrach
	inline void pass_filter(pcl::PointCloud<PointType>::Ptr cloud, float min, float max) {
		boost::mutex mutex;
		mutex.lock();
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
		mutex.unlock();
	}

	//USUWANIE OUTLIERSOW [ile - ilosc branych pod uwage sasiadow, odchylka_od_dewiacji - jaka ma byc max odchylka danego punktu]
	inline void outlier_removal(pcl::PointCloud<PointType>::Ptr cloud, int l_punktow, double odchylka_od_dewiacji) {
		boost::mutex mutex;
		mutex.lock();
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
		mutex.unlock();
	}

	//DOWNSAMPLING VOXEL_GRID - USUWANIE ZBYT DUZEJ ILOSCI PUNKTOW
	inline void downsampling_vox_grid(pcl::PointCloud<PointType>::Ptr cloud, float wielkosc_liscia) {
		boost::mutex mutex;
		mutex.lock();
		if (cloud->empty()) {
			std::cout << "downsampling_vox_grid: Chmura do liczenia jest pusta!" << std::endl;
		}
		else {
				pcl::VoxelGrid<PointType> sor;
				sor.setInputCloud(cloud);
				sor.setLeafSize(wielkosc_liscia, wielkosc_liscia, wielkosc_liscia);
				sor.filter(*cloud);
		}
		mutex.unlock();
	}

	//NORMALNE Z ZASTOSOWANIEM DRZEWA --Promien--Chmura--Normalne-- [radius podaje sie w metrach (3cm to 0.03)]
	inline void normals(pcl::PointCloud<PointType>::Ptr cloud, double promien) {
		if (cloud->empty()) {
			std::cout << "normals: Chmura normalnych albo zwykla jest pusta!" << std::endl;
		}
		else {
			pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
			pcl::NormalEstimation<PointType, pcl::Normal> ne;
			ne.setInputCloud(cloud);
			ne.setRadiusSearch(promien);
			ne.setSearchMethod(tree);
			ne.compute(*cloud_normals);
		}
	}

	//SEGMENTACJA pzrzy użyciu rozrostu obaszarów
	inline void segmentation(pcl::PointCloud<PointType>::Ptr cloud) {
		if (cloud->empty()) {
			std::cout << "segmentation: Chmura do liczenia jest pusta!" << std::endl;
		}
		else {
			// Segmentacja metoda rozrostu obszarow
			pcl::RegionGrowing<PointType, pcl::Normal> segmentacja;
			pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
			tree->setInputCloud(cloud);
			//Ustawienie parametrów Region Growingu
			segmentacja.setMinClusterSize(95);
			segmentacja.setMaxClusterSize(10000);
			segmentacja.setSearchMethod(tree);
			segmentacja.setNumberOfNeighbours(30);
			segmentacja.setInputCloud(cloud);
			segmentacja.setInputNormals(cloud_normals);
			// Ustawienie kata w radianach
			segmentacja.setSmoothnessThreshold(5.0 / 180.0 * M_PI); // 7 stopni.
			segmentacja.setCurvatureThreshold(0.7);
			std::vector <pcl::PointIndices> segmenty;
			segmentacja.extract(segmenty); // To rozpoczyna cały zainicjowany wcześniej proces, zwraca tablicę segmentów
			int currentClusterNum = 1;
			for (std::vector<pcl::PointIndices>::const_iterator i = segmenty.begin(); i != segmenty.end(); ++i)
			{
				// Dodanie wszystkich punktów do nowej chmury
				pcl::PointCloud<PointType>::Ptr segment(new pcl::PointCloud<PointType>);
				for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
					segment->points.push_back(cloud->points[*point]);
				segment->width = segment->points.size();
				segment->height = 1;
				segment->is_dense = true;
				if (segment->size() > 3000) segment->clear();
				// Zapis na dysk
				if (segment->points.size() <= 0)
					break;
				std::cout << "Klaster " << currentClusterNum << " zawiera " << segment->points.size() << " punktow." << std::endl;
				std::string fileName = "Klaster" + boost::to_string(currentClusterNum) + ".pcd";
				pcl::io::savePCDFileASCII(fileName, *segment);
				currentClusterNum++;
			}
			///pcl::PointCloud <PointType>::Ptr colored_cloud = segmentacja.getColoredCloud();
			//cloud->swap(*colored_cloud); //JAK SIE ZMIENI NA RGBA TO TRZEBA 
		}
	}

	//SEGMENTACJA Euclidean Cluster (ta, która powinna tu zostać użyta)
	inline void segmentationV2(pcl::PointCloud<PointType>::Ptr cloud) {
		if (cloud->empty()) {
			cout << "SegmentationV2: Chmura jest pusta!"<<std::endl;
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

			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
			{
				pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
				for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
					cloud_cluster->points.push_back(cloud->points[*pit]); //*
				}
				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;
			}
		}
	}

	//----------------C: PRZETWARZANIE---------------------------

};



int main(int argc, char* argv[])
{
	//PROGRAM NIE DZIAŁA W BLOKU TRY...CATCH, COMPILE-TIME ERROR
	/*try { 
		Kinect kinect;
		kinect.runThreads();
	}
	catch (std::exception & ex) {
		std::cout << ex.what() << std::endl;
	}*/


	Kinect kinect;
	kinect.runThreads();

	time_t stop = clock();
	std::cout << "Czas wykonywania: " << (double)(stop - start) / CLOCKS_PER_SEC << std::endl;
	std::cout << "Podczas tego liczba klatek wyniosla: " << ile_klatek << std::endl;
	std::cout << "liczba FPS podczas calego dzialania :" << (double)(ile_klatek / (double)(stop - start)) * CLOCKS_PER_SEC << std::endl;

	return 0;
}