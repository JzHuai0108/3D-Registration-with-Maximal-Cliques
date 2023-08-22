// align coloradar sequences by brute force matching
#include <iostream>

namespace Eigen {
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
}

std::vector<std::string> findPcds(const std::string &dir) {
    // find all pcds in a directory
    std::vector<std::string> pcds;
    DIR *dp;
    struct dirent *dirp;
    if ((dp = opendir(dir.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return pcds;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string filename = std::string(dirp->d_name);
        if (filename.find(".pcd") != std::string::npos) {
            pcds.emplace_back(filename);
        }
    }
    closedir(dp);
    return pcds;
}

void convertRosbagToPcd(const std::string& rosbag_dir, const std::vector<std::vector<std::string>> &groups,
     const std::string& pcd_dir) {
    // convert pointcloud2 messages on topic /coloradar/points to pcd files
    // save pcd files to pcd_dir
    std::string lidartopic = "/os1_cloud_node/points";
    for (const auto &group : groups) {
        for (const std::string &seq : group) {
            std::string output_seq_dir = pcd_dir + "/" + seq;
            mkdir(output_seq_dir.c_str(), 0777);
            std::string bagname = rosbag_dir + "/" + seq + ".bag";
            rosbag::Bag bag;
            bag.open(bagname, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.emplace_back(lidartopic);
            rosbag::View view(bag, rosbag::TopicQuery(topics));
            int i = 0;
            int j = 0;
            for (const auto &msg : view) {
                sensor_msgs::PointCloud2::ConstPtr pointcloud2_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                if (i % 10 == 0) {
                    std::string filename = output_seq_dir + "/" + std::to_string(j) + ".pcd";
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::fromROSMsg(*pointcloud2_msg, *cloud);
                    pcl::io::savePCDFileASCII(filename, *cloud);
                    ++j;
                }
                ++i;
            }
        }
    }
}

std::pair<double, double> dist(const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2) {
    // return the distance between two transformations
    Eigen::Matrix4d t = t1.inverse() * t2;
    Eigen::Vector3d r = t.block<3, 1>(0, 3);
    Eigen::Matrix3d q = t.block<3, 3>(0, 0);
    Eigen::AngleAxisd aa(q);
    return std::make_pair(r.norm(), aa.angle());
}

void assignToClosestTransform(Eigen::AlignedVector<Eigen::AlignedVector<Eigen::Matrix4d>> &grouped_transforms, 
    const Eigen::Matrix4d &transform) {
    int g = -1;
    for (auto &group : grouped_transforms) {
        std::pair<double, double> d = dist(group[0], transform);
        if (d.first < 0.1 && d.second < 0.05) {
            g = &group - &grouped_transforms[0];
            break;
        }
    }
    if (g == -1) {
        grouped_transforms.emplace_back(Eigen::AlignedVector<Eigen::Matrix4d>);
        grouped_transforms.back().emplace_back(transform);
    } else {
        grouped_transforms[g].emplace_back(transform);
    }
}

bool foundCorrectTransform(const Eigen::AlignedVector<Eigen::AlignedVector<Eigen::Matrix4d>> &grouped_transforms) {
    for (auto &group : grouped_transforms) {
        if (group.size() > 10) {
            std::cout << "found correct transform" << std::endl;
            std::cout << group[0] << std::endl;
            return true;
        }
    }
    return false;
}


void alignTwoSeqs(const std::string &base_dir, const std::string &query_dir) {
    std::cout << "aligning " << query_dir << " to " << base_dir << std::endl;
    std::vector<std::string> base_pcds = findPcds(base_dir);
    std::vector<std::string> query_pcds = findPcds(query_dir);

    std::random_shuffle(base_pcds.begin(), base_pcds.end());
    std::random_shuffle(query_pcds.begin(), query_pcds.end());

    Eigen::AlignedVector<Eigen::AlignedVector<Eigen::Matrix4d>> grouped_transforms;
    // each component is a list of very close transforms


    for (const auto& base_pcd : base_pcds) {
        for (const auto& query_pcd : query_pcds) {
            // TODO: yiwen align base_pcd to query_pcd, refer to demo().

            // TODO: if the error metric is very good
                assignToClosestTransform(grouped_transforms, transform);
            
            bool found = foundCorrectTransform(grouped_transforms);
            if (found)
                break;
        }
    }
}



void alignAllSeqs(const std::string& pcd_dir, const std::vector<std::vector<std::string>> &groups) {

    for (const auto& group : groups) {
        int j = 0;
        for (const auto& seq : group) {
            if (j == 0)
                continue;
            alignTwoSeqs(pcd_dir + "/" + group[0], pcd_dir + "/" + seq);
            ++j;
        }
    }
}

int main(int argc, char** argv) {
    std::cout << "Hello, World!" << std::endl;
    bool has_pcd = false;
    if (argc < 2) {
        std::cout << "Usage: align_coloradar_seqs <rosbag_dir> <pcd_dir>" << std::endl;
        return 1;
    }
    if (argc == 3) {
        has_pcd = true;
    }
    std::string rosbag_dir = argv[1];
    std::string pcd_dir = argv[2];
    std::vector<std::vector<std::string>> groups = {
        {"arpg_lab_run0",
        "arpg_lab_run1",
        "arpg_lab_run2",
        "arpg_lab_run3",
        "arpg_lab_run4",
        }, {
        "aspen_run0",
        // TODO: add other filenames
        }
    };

    convertRosbagToPcd(rosbag_dir, groups, pcd_dir);

    alignAllSeqs(pcd_dir, groups);

    return 0;
}