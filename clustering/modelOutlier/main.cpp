#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cout << "Loading file" << endl;
    pcl::io::loadPCDFile("../../../resources/table.pcd", *cloud);
    cerr << "Cloud before filtering: " <<cloud->points.size()<< endl;


    pcl::ModelCoefficients sphere_coeff;
    sphere_coeff.values.resize(4);
    sphere_coeff.values[0] = 0;
    sphere_coeff.values[1] = 0;
    sphere_coeff.values[2] = 0;
    sphere_coeff.values[3] = 1;

    pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
    sphere_filter.setModelCoefficients(sphere_coeff);
    sphere_filter.setThreshold(0.5);
    sphere_filter.setModelType(pcl::SACMODEL_SPHERE);
    sphere_filter.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sphere_filter.filter(*cloud_sphere_filtered);

    cout << "Done Filtering" << endl;
    cerr << "Cloud after filtering: " <<cloud_sphere_filtered->points.size()<< endl;
 

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_sphere_filtered);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}