#include <liblas/liblas.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file.las>" << std::endl;
        return -1;
    }

    std::ifstream ifs;
    ifs.open(argv[1], std::ios::in | std::ios::binary);
    if (!ifs) {
        std::cerr << "Error opening file." << std::endl;
        return -1;
    }

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    while (reader.ReadNextPoint()) {
        liblas::Point const& p = reader.GetPoint();
        pcl::PointXYZI point;
        point.x = p.GetX();
        point.y = p.GetY();
        point.z = p.GetZ();
        point.intensity = p.GetIntensity();
        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    std::cout << "Loaded " << cloud->points.size() << " points from the LAS file." << std::endl;

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(1.0f, 1.0f, 1.0f);  // Adjust based on density
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_filter.filter(*filtered_cloud);

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 100.0);  // Adjust to limit height range
    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pass.filter(*height_filtered_cloud);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(height_filtered_cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(height_filtered_cloud, intensity_distribution, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    viewer->initCameraParameters();
    viewer->addCoordinateSystem(1.0);

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}
