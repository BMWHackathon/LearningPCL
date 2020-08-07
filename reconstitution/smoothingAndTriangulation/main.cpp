#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cout << "Loading file" << endl;
    pcl::io::loadPCDFile("../../../resources/bunny.pcd", *cloud);
    cout << "Choose method: (1) Smoothing, (2) Triangulation, (3) Normal cloud: ";
    int mode;
    cin >> mode;
    cout << "Chosen method: " << mode << endl;

    if (mode == 1)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        // Output has the PointNormal type in order to store the normals calculated by MLS
        pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);

        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        mls.setComputeNormals(true); //optional
        mls.setInputCloud(cloud);
        mls.setPolynomialOrder(2);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(0.03); // Use all neighbors in a radius of 3cm.
                                   //filter.setPolynomialFit(true); //If true, the surface and normal are approximated using a polynomial estimation
                                   // (if false, only a tangent one).
                                   // Reconstruct
        mls.process(*smoothedCloud);

        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        pcl::PointCloud<pcl::PointXYZ>::Ptr smoothedCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);

        smoothedCloudXYZ->resize(smoothedCloud->points.size());
        for (size_t i = 0; i < smoothedCloud->points.size(); i++)
        {
            const pcl::PointNormal &mls_pt = smoothedCloud->points[i];
            pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);
            smoothedCloudXYZ->points.push_back(pt);
        }
        viewer.addPointCloud<pcl::PointXYZ>(smoothedCloudXYZ);
        while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }
    }
    else if (mode == 2)
    {

        // Object for storing the normals.
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        // Object for storing both the points and the normals.
        pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
        // Normal estimation.
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
        normalEstimation.setInputCloud(cloud);
        normalEstimation.setRadiusSearch(0.03);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        normalEstimation.setSearchMethod(kdtree);
        normalEstimation.compute(*normals);

        // The triangulation object requires the points and normals to be stored in the same structure.
        pcl::concatenateFields(*cloud, *normals, *cloudNormals);
        // Tree object for searches in this new object.
        pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointNormal>);
        kdtree2->setInputCloud(cloudNormals);

        // Triangulation object.
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> triangulation;
        // Output object, containing the mesh.
        pcl::PolygonMesh triangles;
        // Maximum distance between connected points (maximum edge length).
        triangulation.setSearchRadius(0.025);
        // Maximum acceptable distance for a point to be considered,
        // relative to the distance of the nearest point.
        triangulation.setMu(2.5);
        // How many neighbors are searched for.
        triangulation.setMaximumNearestNeighbors(100);
        // Points will not be connected to the current point
        // if their normals deviate more than the specified angle.
        triangulation.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees.
        // If false, the direction of normals will not be taken into account
        // when computing the angle between them.
        triangulation.setNormalConsistency(false);
        // Minimum and maximum angle there can be in a triangle.
        // The first is not guaranteed, the second is.
        triangulation.setMinimumAngle(M_PI / 18);    // 10 degrees.
        triangulation.setMaximumAngle(2 * M_PI / 3); // 120 degrees.

        // Triangulate the cloud.
        triangulation.setInputCloud(cloudNormals);
        triangulation.setSearchMethod(kdtree2);
        triangulation.reconstruct(triangles);

        //triangles can be set as vtk file
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.addPolygonMesh(triangles);
        while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }
    }
    else if (mode == 3)
    {

        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.addPointCloud<pcl::PointXYZ>(cloud);
        while (!viewer.wasStopped())
        {
            viewer.spinOnce();
        }
    }
    else
    {
        cout << "Terminated" << endl;
        return 0;
    }
    return 0;
}