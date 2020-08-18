#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    cout << "Loading file" << endl;
    pcl::io::loadPCDFile("../../../resources/table.pcd", *cloud);

    cout << "Choose method: (1) Statistical Outlier Removal, (2) Radius Outlier removal: ";
    int mode;
    cin >> mode;
    cout << "Chosen method: " << mode << endl;
    cout << "Filtering" << endl;
    cout<< "Size before filtering: "<<cloud->points.size()<<endl;
    if (mode == 1)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);    //input
        sor.setMeanK(50);            //Neighboring points considered during analysis (50)
        sor.setStddevMulThresh(1.0); // Distance information to be processed by outlier
        sor.filter(*cloud_filtered); // Apply filter
    }
    else if (mode == 2)
    {

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.01);       //Search range 0.01
        outrem.setMinNeighborsInRadius(10); // Minimum number of points 10
        outrem.filter(*cloud_filtered);     // Apply filter
    }
    else
    {

        cout << "terminated" << endl;
        return 0;
    }
cout<<"Size after filtering: "<<cloud_filtered->points.size()<<endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    cout << "Done" << endl;
    return 0;
}
