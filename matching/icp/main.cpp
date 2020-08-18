#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
using namespace std;
int main()
{

  cout << "Choose mode: (1) Generated Cloud, (2) Loaded Cloud: " ;
  int mode;
  cin >> mode;
  cout << "Mode chosen: " << mode << endl;

   pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  
  if (mode == 1)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZ>(5,1));
    for (auto &point : *cloud_in1)
    {    
      point.x = 1024 * rand() / (RAND_MAX + 1.0f);
      point.y = 1024 * rand() / (RAND_MAX + 1.0f);
      point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    *cloud_in = *cloud_in1;
    cout << "Saved " << cloud_in->points.size() << " data points to input:" << endl;

    for (auto &point : *cloud_in)
      cout << point << endl;

    *cloud_out = *cloud_in;

    cout << "size:" << cloud_out->points.size() << endl;
    for (auto &point : *cloud_out)
      point.x += 0.7f;

    cout << "Transformed " << cloud_in->points.size() << " data points:" << endl;

    for (auto &point : *cloud_out)
      cout << point << endl;
  }
  else if (mode == 2)
  {

    pcl::io::loadPCDFile<pcl::PointXYZ>("../../../resources/table.pcd", *cloud_in);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../../../resources/tableReduced.pcd", *cloud_out);

  }


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
