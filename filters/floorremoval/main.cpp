
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
int main()
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
        cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>),
        inlierPoints(new pcl::PointCloud<pcl::PointXYZRGB>),
        inlierPoints_neg(new pcl::PointCloud<pcl::PointXYZRGB>);
    cout << "Loading file" << endl;
    pcl::io::loadPCDFile("../../../resources/tabletop.pcd", *cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    cout << "Segmenting" << endl;
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);     // Enable model coefficient refinement (optional).
    seg.setInputCloud(cloud);              //input
    seg.setModelType(pcl::SACMODEL_PLANE); // Configure the object to look for a plane.
    seg.setMethodType(pcl::SAC_RANSAC);    // Use RANSAC method.
    seg.setMaxIterations(1000);            //Maximum number of executions
    seg.setDistanceThreshold(0.01);        // Distance information to be processed as inlier // Set the maximum allowed distance to the model.
    //seg.setRadiusLimits(0, 0.1);     // cylinder, Set minimum and maximum radii of the cylinder.
    seg.segment(*inliers, *coefficients);

    cout << "Segmenting done" << endl;
    cerr << "Model coefficients: " << coefficients->values[0] << " "
         << coefficients->values[1] << " "
         << coefficients->values[2] << " "
         << coefficients->values[3] << endl;
    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *inliers, *inlierPoints);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); //false
    extract.filter(*inlierPoints_neg);

    //Floor
    pcl::visualization::PCLVisualizer viewer("Floor");
    viewer.addPointCloud<pcl::PointXYZRGB>(inlierPoints);

    //Table
    pcl::visualization::PCLVisualizer viewer2("Floor removed");
    viewer2.addPointCloud<pcl::PointXYZRGB>(inlierPoints_neg);

    //Original
    pcl::visualization::PCLVisualizer viewer3("Original");
    viewer3.addPointCloud<pcl::PointXYZRGB>(cloud);
    while (!viewer.wasStopped() || !viewer2.wasStopped() || !viewer3.wasStopped())
    {
        viewer.spinOnce();
    }
    cout << "Done" << endl;
    return 0;
}