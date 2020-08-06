
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;



int main(){

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    cout << "Loading file" << endl;
    pcl::io::loadPCDFile("../../../resources/lamppost.pcd", *cloud);
cout<<"Cloud before filtering: "<<cloud->points.size()<<endl;
cout<<"Filtering"<<endl;
  pcl::PointIndicesPtr ground (new pcl::PointIndices);
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf . setMaxWindowSize ( 20 ) ;
  pmf . setSlope ( 1.0f ) ;
  pmf . setInitialDistance ( 0.5f ) ;
  pmf . setMaxDistance ( 3.0f ) ;
  pmf.extract (ground->indices);

/*
cout<<"Indices: "<<ground->indices[0]<<endl;
cout<<"Indices: "<<ground->indices[1]<<endl;
cout<<"Indices: "<<ground->indices[2]<<endl;
cout<<"Indices: "<<ground->indices[3]<<endl;
cout<<"Indices size: "<<ground->indices.size()<<endl;
*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);  
  //extract.setNegative (true);  //samp11-utm_object.pcd 
  extract.filter (*cloud_filtered);
cout<<"Cloud afterfiltering: "<<cloud_filtered->points.size()<<endl;

  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered);
        while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }

return 0;


}