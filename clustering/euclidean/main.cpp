#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

//There is condition euclidean too
using namespace std;

int main()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    cout << "Loading file" << endl;
    pcl::io::loadPCDFile("../../../resources/table_scene_lms400.pcd", *cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std ::vector<pcl ::PointIndices> cluster_indices; // Index of clustered results, multiple clustering objects in cluster_indices[0]
    // Create clustering object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    cout << "Extracting Clusters" << endl;
    ec.setInputCloud(cloud);      // input
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);    // Minimum number of points
    ec.setMaxClusterSize(25000);  // maximum number of points
    ec.setSearchMethod(tree);     // Specify the search method defined above
    ec.extract(cluster_indices);  // Apply clustering
                                  // Collect, print, and store information for each cluster
    int j = 0;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        // Output the number of points
        cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << endl;
        // Create and save name for each cluster
        stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        //pcl::PCDWriter writer;
        //writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.addPointCloud<pcl::PointXYZ>(cloud_cluster);
        while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }

        j++;
    }
    return 0;
}
