#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main()
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../../resources/cloud_cluster_0.pcd", *cloud);

    // Color unification for visual confirmation (255,255,255)
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;
    }

    //KdTree object creation
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);

    // How to set the reference point (searchPoint) #1 (specify x,y,z coordinates)
    //pcl::PointXYZRGB searchPoint;
    //searchPoint.x = 0.026256f;
    //searchPoint.y = -1.464739f;
    //searchPoint.z = 0.929567f;
    //How to set the reference point (searchPoint) #2 (3000th point)
    pcl::PointXYZRGB searchPoint = cloud->points[3000];

    //Search for a point from the reference point to the K-th closest order (K ​​nearest neighbor search)
    int K = 10; // Set the number of points to search
    vector<int> pointIdxNKNSearch(K);
    vector<float> pointNKNSquaredDistance(K);

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        //Change color for visual confirmation (0,255,0)
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        {
            cloud->points[pointIdxNKNSearch[i]].r = 0;
            cloud->points[pointIdxNKNSearch[i]].g = 255;
            cloud->points[pointIdxNKNSearch[i]].b = 0;
        }
    }

    // Output the number of points searched
    cout << "K = 10 ：" << pointIdxNKNSearch.size() << endl;

    // Neighbor search within radius from the reference point
    float radius = 0.02; //Set the search radius
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        //Change color for visual confirmation (0,0,255)
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            {
                cloud->points[pointIdxRadiusSearch[i]].r = 0;
                cloud->points[pointIdxRadiusSearch[i]].g = 0;
                cloud->points[pointIdxRadiusSearch[i]].b = 255;
            }
    }

    // Output the number of points searched
    cout << "Radius 0.02 nearest neighbors: " << pointIdxRadiusSearch.size() << endl;

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}