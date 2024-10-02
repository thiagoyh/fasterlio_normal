#include <iostream>
#include <filesystem>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


int main(int argc, char** argv) {
    if (argc < 2) return 0;
    std::string pcd_folder = argv[1];
    if (std::filesystem::exists(pcd_folder) && std::filesystem::is_directory(pcd_folder)) {
        // 遍历目录中的每个文件和子目录
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr map(new pcl::PointCloud<pcl::PointXYZINormal>);
        for (const auto& entry : std::filesystem::directory_iterator(pcd_folder)) {
            std::string pcd_file = entry.path().string();
            if (pcd_file.find(".pcd") != std::string::npos) {
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
                if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_file, *cloud) == -1) {
                    std::cerr << "Failed to read " << pcd_file << std::endl;
                    continue;
                }
                std::cout << "read pcd file: " << pcd_file << std::endl;
                *map += *cloud;
            }
        }
        std::string all_points_dir_ply = pcd_folder + "/all_points.ply";
        if (pcl::io::savePLYFile(all_points_dir_ply, *map) == -1) {
            return -1;
        }

        std::string all_points_dir = pcd_folder + "/all_points.pcd";
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *map);

        std::string pcd_ascii = pcd_folder + "/all_points_ascii.pcd";
        if (pcl::io::savePCDFileASCII(pcd_ascii, *map) == -1) {
            return -1;
        }
    }

    return 0;
}
