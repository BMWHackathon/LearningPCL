#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;
int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    // read *.PCD file (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop.pcd)
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../../resources/tabletop.pcd", *cloud);
    // Output the number of points
    cout
        << "Loaded: " << cloud->width * cloud->height << endl;


    cout << "Choose method: (1) PassThrough, (2) Conditional Removal: ";
    int mode;
    cin >> mode;
    cout << "Chosen method: " << mode << endl;
    cout << "Filtering" << endl;

    if (mode == 1)
    {
        // create object
        pcl::PassThrough<pcl::PointXYZRGB>
            pass;
        pass.setInputCloud(cloud);       //input
        pass.setFilterFieldName("z");    //Coordinate axis to be applied (eg. Z axis)
        pass.setFilterLimits(0.70, 1.5); //Value to apply (min, max)
        //pass.setFilterLimitsNegative (true); //Other values ​​to apply
        pass.filter(*cloud_filtered); // Apply filter
                                      // Output the number of points
    }
    else if (mode == 2)
    {
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 0.8)));
        //Create object
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
        condrem.setInputCloud(cloud);     //입력
        condrem.setCondition(range_cond); // condition setting
        condrem.setKeepOrganized(true);   //
        condrem.filter(*cloud_filtered);  // Apply filter
    }else{
        cout<<"terminating"<<endl;
        return 0;
    }

    cout
        << "Filtered: " << cloud_filtered->width * cloud_filtered->height << endl;

    // Save
    //pcl::io::savePCDFile<pcl::PointXYZRGB>("tabletop_passthrough.pcd", *cloud_filtered); //Default binary mode save
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_filtered);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return (0);
}