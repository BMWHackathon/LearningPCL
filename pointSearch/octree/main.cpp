#include <pcl/io/pcd_io.h> 
#include <pcl/octree/octree_search.h>  
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/point_types.h>  
#include <iostream>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;


int main(){

    cout<<"Loading file"<<endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("../../../resources/tabletop.pcd", *cloud);

    cout<<"Coloring cloud"<<endl;
    for (size_t i = 0; i < cloud->points.size(); ++i){
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;
    }


float resolution = 0.03f ; //Set voxel size (Set octree voxel resolution)  
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution); 
    octree.setInputCloud(cloud);      
    octree.addPointsFromInputCloud();  //Octree  (Build Octree)


    // How to set the reference point (searchPoint) #1 (specify x,y,z coordinates)
    //pcl::PointXYZRGB searchPoint;
    //searchPoint.x = 0.026256f;
      //searchPoint.y = -1.464739f;
      //searchPoint.z = 0.929567f;


        //How to set the reference point (searchPoint) #2 (3000th point)
    pcl::PointXYZRGB searchPoint = cloud->points[3000]; 


     //Output the reference point coordinates 
     cout << "searchPoint :" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z  << endl;


          //Voxel Neighbor Search in the same voxel as the reference point
    vector<int> pointIdxVec;  // Index (Save the result vector of the voxel neighbor search) 

        if (octree.voxelSearch(searchPoint, pointIdxVec))
    {
        //Change color for visual confirmation (255,0,0)
        for (size_t i = 0; i < pointIdxVec.size(); ++i){
            cloud->points[pointIdxVec[i]].r = 255;
            cloud->points[pointIdxVec[i]].g = 0;
            cloud->points[pointIdxVec[i]].b = 0;
        }        
    }
  // Output the number of points searched 
    cout << "Voxel search points: " << pointIdxVec.size() << endl;


        // Search for a point from the reference point to the K-th closest order (K ​​nearest neighbor search)
    int K = 50 ; // Set the number of points to search    
    vector<int> pointIdxNKNSearch; //Save the index result of the K nearest neighbor
    vector<float> pointNKNSquaredDistance;  //Save the index result of the K nearest neighbor
    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {   
          //Change color for visual confirmation (0,255,0)
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
            cloud->points[pointIdxNKNSearch[i]].r = 0;
            cloud->points[pointIdxNKNSearch[i]].g = 255;
            cloud->points[pointIdxNKNSearch[i]].b = 0;
        }    
    }

         // Output the number of points searched 
    cout << "K = 50 nearest neighbors:" << pointIdxNKNSearch.size() << endl;


// Neighbor search within radius
    float radius = 0.02 ; //Set the search radius  
     vector<int> pointIdxRadiusSearch;  //Save the index of each neighbor
    vector<float> pointRadiusSquaredDistance;  //Save the square of the Euclidean distance between each neighbor and the search point

    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {    
          //Change color for visual confirmation (0,0,255)
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
            cloud->points[pointIdxRadiusSearch[i]].r = 0;
            cloud->points[pointIdxRadiusSearch[i]].g = 0;
            cloud->points[pointIdxRadiusSearch[i]].b = 255;
        }        
    }

    // Output the number of points searched 
    cout << "Radius 0.02 nearest neighbors: " << pointIdxRadiusSearch.size() << endl;

   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud);
        while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }
    return 0;
}