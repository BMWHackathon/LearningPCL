#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <string>

using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
   
    pcl::io::loadPCDFile("../../../resources/bunny.pcd", *cloud);

    //Normal estimation

            // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree2);

    // cloud_normals is the output datasets

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*normals);



    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud(cloud);
    vfh.setInputNormals(normals);
    // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    vfh.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

    // Compute the features
    vfh.compute(*vfhs);

    cout<<"DONE: "<<vfhs->points.size()<<endl;

    pcl::visualization::PCLHistogramVisualizer viewer;
    string name = "VFH";
    viewer.addFeatureHistogram(*vfhs,300);
  
     while (1)
    {
        viewer.spinOnce();
    }
    return 0;
}