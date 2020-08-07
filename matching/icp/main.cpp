#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
using namespace std;
int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../../../resources/table.pcd", *cloud_in);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../../../resources/tableReduced.pcd", *cloud_out);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>
            icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    cout << "Has converged:" << icp.hasConverged() << " Score: " << icp.getFitnessScore() << endl;
    cout << icp.getFinalTransformation() << endl;

    return 0;
}