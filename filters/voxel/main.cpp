#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    cout << "Loading file" << endl;
    pcl::io::loadPCDFile("../../../resources/table.pcd", *cloud);
    cout << "Choose method: (1) VoxelGrid, (2) Uniform sampling: ";
    int mode;
    cin >> mode;
    cout << "Chosen method: " << mode << endl;
    cout << "Filtering" << endl;
    if (mode == 1)
    {

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloud_filtered);

        // Save the created point cloud
        //pcl :: io :: savePCDFile < pcl :: PointXYZ > ( "table_scene_lms400_downsampled.pcd" , * cloud_filtered ) ;
    }
    else if (mode == 2)
    {

        pcl::UniformSampling<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud);
        filter.setRadiusSearch(0.01f);
        filter.filter(*cloud_filtered);
    }
    else
    {
        cout << "terminating" << endl;
        return 0;
    }

    cout << "Done filtering" << endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    cout << "Done" << endl;
    return (0);
}