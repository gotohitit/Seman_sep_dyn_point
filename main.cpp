#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <filename.pcd>" << std::endl;
        return -1;
    }

    // 构造PCD文件路径
    std::string input_file = "../PCD/" + std::string(argv[1]);

    // 创建点云对象并读取PCD文件
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(input_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", input_file.c_str());
        return -1;
    }
    std::cout << "Loaded " << cloud->points.size() << " data points from " << input_file << std::endl;

    // 遍历点云并修改强度值
    for (auto& point : cloud->points) {
        uint32_t intensity = static_cast<uint32_t>(point.intensity);
        int label = intensity & 0xFFFF;
        if (label <= 259 && label >= 252) {
            point.intensity = 240;
        } else {
            point.intensity = 10;
        }
    }

    // 保存修改后的点云到新文件
    std::string output_file = "../PCD_OUT/modified_" + std::string(argv[1]);
    if (pcl::io::savePCDFileASCII(output_file, *cloud) == -1) {
        PCL_ERROR("Couldn't write file %s\n", output_file.c_str());
        return -1;
    }

    std::cout << "Saved modified cloud to " << output_file << std::endl;
    return 0;
}
