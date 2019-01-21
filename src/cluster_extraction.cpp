// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <sstream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/common/pca.h>
#include <pcl/surface/mls.h>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>


typedef pcl::PointXYZRGB PointXYZRGB;

int main(int argc, char* argv[])
{

	// Point Cloud
	pcl::PointCloud<PointXYZRGB>::ConstPtr cloud;
	pcl::PointCloud<PointXYZRGB>::Ptr cloud_j(new pcl::PointCloud<PointXYZRGB>());
	

	cout << "Cloud initialized." << endl;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointXYZRGB>::ConstPtr&)> pointcloud_function =
		[&cloud, &mutex](const pcl::PointCloud<PointXYZRGB>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);


		cloud = ptr;
		//cout << "Callback function called." << endl;
	};

	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
	cout << "Grabber initialized." << endl;

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(pointcloud_function);
	cout << "Callback function registered." << endl;

	// Keyboard Callback Function 
	boost::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_function =
		[&cloud, cloud_j, &mutex](const pcl::visualization::KeyboardEvent& event) {
		if (event.getKeyCode() == VK_SPACE && event.keyDown()) {

			// Save Point Cloud to PCD File when Pressed Space Key

			boost::mutex::scoped_try_lock lock(mutex);
			if (lock.owns_lock()) {

				pcl::io::savePCDFileBinary("cloud_new.pcd", *cloud_j);
				std::cerr << "Saved " << cloud_j->points.size() << " data points to cloud_new.pcd." << std::endl;

			}
		}
	};

	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer(argc, argv, "Point Cloud Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();

	// Register Keyboard Callback Function
	viewer->registerKeyboardCallback(keyboard_function);
	cout << "Callback Keyboard Function" << endl;

	// Start Grabber
	grabber->start();
	cout << "Grabber started." << endl;

	while (!viewer->wasStopped()) {
		// Update Viewer
		viewer->spinOnce();

		boost::mutex::scoped_try_lock lock(mutex);
		if (cloud && lock.owns_lock()) {
			if (cloud->size() != 0) {

				/* Processing Point Cloud begins here */


				// Create the filtering object
				pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZRGB>());

				// Filtering Box (XYZ)
				pcl::PointCloud<PointXYZRGB>::Ptr cloud_t(new pcl::PointCloud<PointXYZRGB>());
				pcl::PassThrough<PointXYZRGB> pass;
				pass.setInputCloud(cloud);
				pass.setFilterFieldName("z");
				pass.setFilterLimits(0.0, 0.8);
				pass.filter(*cloud_filtered);

				//pcl::PointCloud<PointXYZRGB>::Ptr cloud_g(new pcl::PointCloud<PointXYZRGB>());
				//pass.setInputCloud(cloud_t);
				//pass.setFilterFieldName("y");
				//pass.setFilterLimits(-0.5, 0.0);
				//pass.filter(*cloud_g);

				pcl::PointCloud<PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<PointXYZRGB>());
				pass.setInputCloud(cloud_filtered);
				pass.setFilterFieldName("x");
				pass.setFilterLimits(-0.11, 0.4);
				pass.filter(*cloud_t);

				pcl::PointCloud<PointXYZRGB>::Ptr cloud_v(new pcl::PointCloud<PointXYZRGB>());

				pcl::VoxelGrid<PointXYZRGB> sor;
				sor.setInputCloud(cloud_t);
				sor.setLeafSize(0.0047f, 0.0047f, 0.0047f);
				sor.filter(*cloud_v);

				pcl::PointCloud<PointXYZRGB>::Ptr output(new pcl::PointCloud<PointXYZRGB>);

				// Noise Removal
				pcl::StatisticalOutlierRemoval<PointXYZRGB> kor;
				kor.setInputCloud(cloud_v);
				kor.setMeanK(100);
				kor.setStddevMulThresh(0.15);
				kor.filter(*output);


				//---------------------------------------------------------------------------------------------------------------------

				pcl::search::KdTree<PointXYZRGB>::Ptr tree2(new pcl::search::KdTree<PointXYZRGB>);

				// Output has the PointNormal type in order to store the normals calculated by MLS
				pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

				// Init object (second point type is for the normals, even if unused)
				pcl::MovingLeastSquares<PointXYZRGB, pcl::PointXYZRGBNormal> mls;

				mls.setComputeNormals(true);

				// Set parameters
				mls.setInputCloud(output);
				mls.setPolynomialFit(true);
				mls.setSearchMethod(tree2);
				mls.setSearchRadius(0.03);

				// Reconstructe
			mls.process(mls_points);


				//Create the filtering object
				pcl::copyPointCloud(mls_points, *cloud_p);
				

				//---------------------------------------------------------------------------------------------------------------------


				

				

				// Create the segmentation object
				pcl::SACSegmentation<PointXYZRGB> seg;
				pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
				pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
				// Optional
				seg.setOptimizeCoefficients(true);
				// Mandatory
				seg.setModelType(pcl::SACMODEL_PLANE);
				seg.setMethodType(pcl::SAC_RANSAC);
				seg.setMaxIterations(100);
				seg.setDistanceThreshold(0.01);

				int i = 0, nr_points = (int)cloud_p->points.size();
				while (cloud_p->points.size() > 0.55 * nr_points)
				{
					// Segment the largest planar component from the remaining cloud
					seg.setInputCloud(cloud_p);
					seg.segment(*inliers, *coefficients);
					if (inliers->indices.size() == 0)
					{
						std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
						break;
					}

					// Extract the inliers
					pcl::PointCloud<PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<PointXYZRGB>());
					pcl::ExtractIndices<PointXYZRGB> extract;
					extract.setInputCloud(cloud_p);
					extract.setIndices(inliers);

					

					// Get the points associated with the planar surface
					pcl::PointCloud<PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<PointXYZRGB>()); //Plane Points
					extract.setNegative(false);
					extract.filter(*cloud_plane);
				/*	if (!viewer->updatePointCloud(cloud_plane, "cloud_plane")) {
						viewer->addPointCloud(cloud_plane, "cloud_plane");
						viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_plane");
					}*/

					extract.setNegative(true);
					extract.filter(*cloud_f);
					*cloud_p = *cloud_f;
				}

				if (!viewer->updatePointCloud(cloud_p, "cloud_filtered")) {
					viewer->addPointCloud(cloud_p, "cloud_filtered");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5, "cloud_filtered");
				}

				//Removing Noise
				pcl::StatisticalOutlierRemoval<PointXYZRGB> tor;
				tor.setInputCloud(cloud_p);
				tor.setMeanK(50);
				tor.setStddevMulThresh(1.0);
				tor.filter(*cloud_j);

				

				// Creating the KdTree object for the search method of the extraction
				pcl::search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>);
				tree->setInputCloud(cloud_j);

				std::vector<pcl::PointIndices> cluster_indices;
				pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
				ec.setClusterTolerance(0.02); // 2cm
				ec.setMinClusterSize(550);
				ec.setMaxClusterSize(10000);
				ec.setSearchMethod(tree);
				ec.setInputCloud(cloud_j);
				ec.extract(cluster_indices);
				

				//std::cout << "Number of clusters is equal to " << cluster_indices.size() << std::endl;
			
				pcl::PointCloud<PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<PointXYZRGB>());
				
				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
				{
					cloud_cluster->clear();
					
					for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
						cloud_cluster->points.push_back(cloud_j->points[*pit]); //*
					//std::cout << " ** Number of points in object : " << cloud_cluster->size() << std::endl;
				}

				// Compute principal directions
				Eigen::Vector4f pcaCentroid;
				pcl::compute3DCentroid(*cloud_cluster, pcaCentroid);
				Eigen::Matrix3f covariance;
				computeCovarianceMatrixNormalized(*cloud_cluster, pcaCentroid, covariance);
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
				Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
				eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

				// Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:


				// Ovo vjv nije potrebno
			//	pcl::PointCloud<PointXYZRGB>::Ptr cloudPCAprojection(new pcl::PointCloud<PointXYZRGB>);
			     //pcl::PCA<PointXYZRGB> pca;
				//pca.setInputCloud(cloud_cluster);
				//pca.project(*cloud_cluster, *cloudPCAprojection);
				//std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
			//	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;

				// In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.


				// Transform the original cloud to the origin where the principal components correspond to the axes.
				Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
				projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
				projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
				pcl::PointCloud<PointXYZRGB>::Ptr cloudPointsProjected(new pcl::PointCloud<PointXYZRGB>);
				pcl::transformPointCloud(*cloud_cluster, *cloudPointsProjected, projectionTransform);
				// Get the minimum and maximum points of the transformed cloud.
				PointXYZRGB minP, maxP;
				pcl::getMinMax3D(*cloudPointsProjected, minP, maxP);
				Eigen::Vector3f meanDiagonal = 0.5f*(maxP.getVector3fMap() + minP.getVector3fMap());



				/* Processing Point Cloud ends here */

				// Final transform
				 Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); 
				 Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();


				 viewer->removeAllShapes();
				 viewer->addCube(bboxTransform, bboxQuaternion, maxP.x - minP.x, maxP.y - minP.y, maxP.z - minP.z, "bbox");
			//
				 viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.7, 0, "bbox");
				 viewer->setRepresentationToWireframeForAllActors();
				 viewer->setShowFPS(true);



				 // Isprobati u labu

		//		 float x1, y1, z1;

		//		 maxP.x = x1;
		//		 maxP.y = y1;
		//		 maxP.z = z1;

		//		 std::cout << x1 << std::endl;
		//		 std::cout << y1 << std::endl;
		//		 std::cout << z1 << std::endl;

			//	 std::cout << maxP << std::endl;
		//		 std::cout << maxP.x << std::endl;
		1	//	 std::cout << maxP.y << std::endl;
			//	 std::cout << maxP.z << std::endl;
		//		 std::cout << eigenVectorsPCA << std::endl;
		//		 std::cout << bboxTransform << std::endl;
		//		   std::cout << 
 


				
			     

				 // This viewer has 4 windows, but is only showing images in one of them as written here.
				//pcl::visualization::PCLVisualizer *visu;
				//visu = new pcl::visualization::PCLVisualizer(argc, argv, "PlyViewer");
				//int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
				//visu->createViewPort(0.0, 0.5, 0.5, 1.0, mesh_vp_1);
				//visu->createViewPort(0.5, 0.5, 1.0, 1.0, mesh_vp_2);
				//visu->createViewPort(0.0, 0, 0.5, 0.5, mesh_vp_3);
				//visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
				//visu->addPointCloud(*cloud_cluster, ColorHandlerXYZ(*cloud_cluster, 30, 144, 255), "bboxedCloud", mesh_vp_3);
				//visu->addCube(bboxTransform, bboxQuaternion, maxP.x - minP.x, maxP.y - minP.y, maxP.z - minP.z, "bbox", mesh_vp_3);
				 

				// Update Point Cloud
				if (!viewer->updatePointCloud(cloud_cluster, "cloud_cluster")) {
					viewer->addPointCloud(cloud_cluster, "cloud_cluster");
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.2, "cloud_cluster");		
					//viewer->addCoordinateSystem(1.0);
					cout << "PointCloud updated!" << endl;
					//viewer->resetCameraViewpoint("cloud");
				}

				std::cout << maxP << std::endl;
				std::cout << minP << std::endl;
				std::cout << eigenVectorsPCA << std::endl;
				std::cout << bboxTransform << std::endl;
				std::cout << meanDiagonal << std::endl;
				std::cout << pcaCentroid << std::endl;

						

			}

		}
	}

	// Stop Grabber
	grabber->stop();

	return 0;
}