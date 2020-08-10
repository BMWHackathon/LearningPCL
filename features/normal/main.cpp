#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
int main()
{
    //Loading point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../../resources/bunny.pcd", *cloud);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    //Choose method:
    cout << "Which method: (1) Normal, (2) Integral normal, (3) Raw point cloud: ";
    int mode;
    cin >> mode;
    cout << endl;
    cout << "Chosen method: " << mode << endl;

    if (mode == 1)
    {
        //Normal estimation

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);

        // cloud_normals is the output datasets

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch(0.03);

        // Compute the features
        ne.compute(*cloud_normals);

        // cloud_normals->points.size () should have the same size as the input cloud->points.size ()
    }
    else if (mode == 2)
    {
        //Integral estimation
	cout<<cloud->width<<"  "<<cloud->height<<endl;
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(cloud);
        ne.compute(*cloud_normals);

        /*
        enum NormalEstimationMethod
{
  COVARIANCE_MATRIX,
  AVERAGE_3D_GRADIENT,
  AVERAGE_DEPTH_CHANGE
};
*/
    }
    else if (mode == 3)
    {
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

    cout << "Input Cloud size " << cloud->points.size() << endl;
    cout << "Cloud normals size " << cloud_normals->points.size() << endl;
    cout << "Starting visualization... " << endl;

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    cout << "Done" << endl;
    return 0;
}
