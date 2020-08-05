#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    cout << "Choose method: (1) Non RGB, (2) RGB: ";
    int mode;
    cin >> mode;
    cout << "Chosen method: " << mode << endl;
    if (mode == 1)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

        cout << "Loading file" << endl;
        pcl::io::loadPCDFile("../../../resources/tableReduced.pcd", *cloud);

        // Calculate the surface normal used by the algorithm
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*normals);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(50);
        reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(cloud);
        //reg.setIndices (indices);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(1.0);
        vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        colored_cloud = reg.getColoredCloud();
        cout << "Number of clusters is equal to " << clusters.size() << endl;
        cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
        cout << "These are the indices of the points of the initial" << endl
             << "cloud that belong to the first cluster:" << endl;
    }
    else if (mode == 2)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../../resources/tabletop.pcd", *cloud);

        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
        reg.setInputCloud(cloud);
        reg.setSearchMethod(tree);      // search method
        reg.setDistanceThreshold(10);   // Distance information designated as a neighbor
        reg.setPointColorThreshold(6);  // used to test whether the same Cluter (cf. Just as angle threshold is used for testing points normals)
        reg.setRegionColorThreshold(5); // Used to test whether the same Cluter, used in merging stage
        reg.setMinClusterSize(600);

        vector<pcl::PointIndices> clusters;
        reg.extract(clusters);
        colored_cloud = reg.getColoredCloud();
        cout << "Number of clusters is equal to " << clusters.size() << endl;
        cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
        cout << "These are the indices of the points of the initial" << endl
             << "cloud that belong to the first cluster:" << endl;
    }
    else
    {
        cout << "Terminated" << endl;
        return 0;
    }

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}