#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../../resources/milk.pcd", *cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    cout << "Computing normals" << endl;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree2);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    cout << "Finished computing normals" << endl;

    cout << "Chose the mode: (1) PFH, (2) FPFH, (3) Raw point cloud: ";
    int mode;
    cin >> mode;
    cout << "Chosen mode: " << mode << endl;

    if (mode == 1)
    {

        // Create the PFH estimation class, and pass the input dataset+normals to it
        pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
        pfh.setInputCloud(cloud);
        pfh.setInputNormals(normals);
        // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

        // Create an empty kdtree representation, and pass it to the PFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
        pfh.setSearchMethod(tree);

        // Output datasets
        pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        pfh.setRadiusSearch(0.05);

        //Error prevention
        for (int i = 0; i < normals->points.size(); i++)
        {
            cout << "Checking if finite" << endl;
            cout << "Point: " << normals->points[i] << endl;
            if (!pcl::isFinite<pcl::Normal>(normals->points[i]))
            {

                PCL_WARN("normals[%d] is not finite\n", i);
            }
        }

        // Compute the features
        pfh.compute(*pfhs);

        // pfhs->points.size () should have the same size as the input cloud->points.size ()*
        cout << "PFHS Size: " << pfhs->points.size() << endl;
        cout << "Cloud Size: " << cloud->points.size() << endl;
    }
    else if (mode == 2)
    {
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(normals);
        // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

        // Create an empty kdtree representation, and pass it to the FPFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
         pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

        fpfh.setSearchMethod(tree);

        // Output datasets
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        fpfh.setRadiusSearch(0.05);

        // Compute the features
        fpfh.compute(*fpfhs);
        cout << "FPFHS Size: " << fpfhs->points.size() << endl;
        cout << "Cloud Size: " << cloud->points.size() << endl;
    }else if(mode == 3){
//Raw point cloud

        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.addPointCloud<pcl::PointXYZ>(cloud);
        while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }
        return 0;
    }
    else
    {
        cout << "Terminating" << endl;
        return 0;
    }


    cout << "Done" << endl;

    return 0;
}