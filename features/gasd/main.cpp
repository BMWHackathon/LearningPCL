#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/gasd.h>



using namespace std;



int main(int argc, char** argv) {



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);



    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
        return (-1);

    cout << "Choose method: (1) Colored, (2) Not colored: ";
    int mode;
    cin >> mode;
    cout << "Chosen method: " << mode << endl;


    if (mode == 1) {

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

        pcl::io::loadPCDFile("../../../resources/table.pcd", *cloud);

        // Create the GASD estimation class, and pass the input dataset to it
        pcl::GASDColorEstimation<pcl::PointXYZRGBA, pcl::GASDSignature984> gasd;
        gasd.setInputCloud(cloud);

        // Output datasets
        pcl::PointCloud<pcl::GASDSignature984> descriptor;

        // Compute the descriptor
        gasd.compute(descriptor);

        // Get the alignment transform
        Eigen::Matrix4f trans = gasd.getTransform(trans);

        // Unpack histogram bins
        for (std::size_t i = 0; i < std::size_t(descriptor[0].descriptorSize()); ++i)
        {
            descriptor[0].histogram[i];
        }


    }
    else if (mode == 2) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
            return (-1);

        // Create the GASD estimation class, and pass the input dataset to it
        pcl::GASDEstimation<pcl::PointXYZ, pcl::GASDSignature512> gasd;
        gasd.setInputCloud(cloud);

        // Output datasets
        pcl::PointCloud<pcl::GASDSignature512> descriptor;

        // Compute the descriptor
        gasd.compute(descriptor);

        // Get the alignment transform
        Eigen::Matrix4f trans = gasd.getTransform(trans);

        // Unpack histogram bins
        for (std::size_t i = 0; i < std::size_t(descriptor[0].descriptorSize()); ++i)
        {
            descriptor[0].histogram[i];
        }


    }
    else {


        cout<<"Terminated"<endl;
        return 0;
    }


    cout<<"Done "<<endl;
    return 0;
}