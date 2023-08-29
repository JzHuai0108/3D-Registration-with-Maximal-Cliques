// align coloradar sequences by brute force matching
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <sys/stat.h>
#include <dirent.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "Eva.h"


namespace Eigen {
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
}
bool add_overlap;
bool low_inlieratio;
bool no_logs;

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
    std::string radartopic = "/os1_cloud_node/points";
    for (const auto &group : groups) {
        for (const std::string &seq : group) {
            std::string output_seq_dir = pcd_dir + "/" + seq;
            mkdir(output_seq_dir.c_str(), 0777);
            std::string bagname = rosbag_dir + "/" + seq + ".bag";
            std::cout<<"bagname:"<<bagname<<std::endl;

            rosbag::Bag bag;
            bag.open(bagname, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.emplace_back(radartopic);
            rosbag::View view(bag, rosbag::TopicQuery(topics));
            int i = 0;
            int j = 0;
            std::cout<<"save pcds"<<std::endl;
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
    const Eigen::Matrix4d &transform, auto basepcd, auto querypcd) {
    int g = -1;
    std::cout << "grouped_transforms.size:" << grouped_transforms.size() <<std::endl;
    for (auto &group : grouped_transforms) {
        std::pair<double, double> d = dist(group[0], transform);
        std::cout << "d: " << d.first << ", " << d.second << std::endl;
        if (d.first < 0.1 && d.second < 0.05) {
            g = &group - &grouped_transforms[0];
            ofstream outFile("../result/d_values.txt", ios::out);
            outFile << "base:" << basepcd << " query:" << querypcd << " "
                    << "d: " << d.first << ", " << d.second << " g:" << g<< std::endl;
            break;
        }
    }
    if (g == -1) {
        Eigen::AlignedVector<Eigen::Matrix4d> trans;
        grouped_transforms.emplace_back(trans);
        grouped_transforms.back().emplace_back(transform);
    } else {
        grouped_transforms[g].emplace_back(transform);
    }
}

bool foundCorrectTransform(const Eigen::AlignedVector<Eigen::AlignedVector<Eigen::Matrix4d>> &grouped_transforms,
                           auto basepcd, auto querypcd)
{
    for (auto &group : grouped_transforms) {
        if (group.size() >50) {
            std::cout << "found correct transform" << std::endl;
            std::cout << "group[0]: \n" << group[0] << std::endl;
            std::ofstream outFile("../result/group0.txt");
            outFile << "base:" << basepcd << " query:" << querypcd << "\n" << group[0];
            outFile.close();

            return true;
        }
    }
    return false;
}

void alignTwoSeqs(const std::string &base_dir, const std::string &query_dir) {
    std::cout << "aligning " << query_dir << " to " << base_dir << std::endl;
    std::vector<std::string> base_pcds = findPcds(base_dir);
    std::vector<std::string> query_pcds = findPcds(query_dir);

    srand(time(NULL));
    std::random_shuffle(base_pcds.begin(), base_pcds.end());
    std::random_shuffle(query_pcds.begin(), query_pcds.end());

    Eigen::AlignedVector<Eigen::AlignedVector<Eigen::Matrix4d>> grouped_transforms;
    Eigen::Matrix4d transform;

    ofstream outFile;
    outFile.open("../result/best_scores.txt", ios::out);
    bool found = false;

    for (const auto& base_pcd : base_pcds) {
        for (const auto& query_pcd : query_pcds) {
            PointCloudPtr query_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudPtr base_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudPtr new_query_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudPtr new_base_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            std::string query_pcd_path = query_dir + '/' + query_pcd;
            std::string base_pcd_path = base_dir + '/' + base_pcd;
            std::cout <<"\n" << "#------------------------------#" << std::endl;
            std::cout << "query_pcd:" << query_pcd << " " << "base_pcd:" << base_pcd << std::endl;

            pcl::io::loadPCDFile(query_pcd_path, *query_cloud);
            pcl::io::loadPCDFile(base_pcd_path, *base_cloud);
            std::cout << "query_cloud size: " << query_cloud->size() << std::endl;

            float query_resolution = MeshResolution_mr_compute(query_cloud);
            float base_resolution = MeshResolution_mr_compute(base_cloud);
            float resolution = (query_resolution + base_resolution) / 2;
            std::cout << "resolution" << resolution << std::endl;

            float downsample = 0.6;
            Voxel_grid_downsample(query_cloud, new_query_cloud, downsample);
            Voxel_grid_downsample(base_cloud, new_base_cloud, downsample);

            vector<vector<float>> query_feature, base_feature;
            FPFH_descriptor(new_query_cloud, downsample * 5, query_feature);
            FPFH_descriptor(new_base_cloud, downsample * 5, base_feature);

            vector<Corre_3DMatch> correspondence;
            feature_matching(new_query_cloud, new_base_cloud, query_feature, base_feature, correspondence);

            std::cout<<"correspondence.size:"<< correspondence.size()<<"\n";

            vector<double> ov_lable;
            ov_lable.resize((int)correspondence.size());

            string folderPath = "../result";
            double best_score = 0;
            cout << "Start registration." << endl;
            coloradar_registration(query_cloud, base_cloud, correspondence, ov_lable,
                                   transform, folderPath, resolution, 0.99, best_score);
            std::cout << "Transform matrix: \n" << transform << std::endl;
            cout<<"best_score:"<<best_score<< endl;
            outFile << base_pcd << " " << query_pcd << " " << best_score << endl;

            if (best_score > 30){
                std::cout << "start assignToClosestTransform" <<std::endl;
                assignToClosestTransform(grouped_transforms, transform, base_pcd, query_pcd);
            }
            std::cout << "start foundCorrectTransform" <<std::endl;
            found = foundCorrectTransform(grouped_transforms, base_pcd, query_pcd);
            // clear data
            query_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            base_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            new_query_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            new_base_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            query_feature.clear();
            query_feature.shrink_to_fit();
            base_feature.clear();
            base_feature.shrink_to_fit();
            correspondence.clear();
            correspondence.shrink_to_fit();
            ov_lable.clear();
            ov_lable.shrink_to_fit();
            
            if (found){
                std::cout << "found" <<std::endl;
                break;
            }
        }
        if(found){
            break;
        }
    }
}


void alignAllSeqs(const std::string& pcd_dir, const std::vector<std::vector<std::string>> &groups) {
    std::cout << "alignAllSeqs" << std::endl;
    for (const auto &group : groups){
        int j = 0;
        for (const auto &seq : group){
            if (j == 0){
                ++j;
                continue;
            }
            std::string base_pcds = pcd_dir + "/" + group[0];
            std::string query_pcds = pcd_dir + "/" + seq;
            alignTwoSeqs(base_pcds, query_pcds);
            ++j;
        }
    }
}

int main(int argc, char** argv) {
    std::cout << "Hello, World!" << std::endl;
    bool has_pcd = true;
    if (argc < 2) {
        std::cout << "Usage: align_coloradar_seqs <rosbag_dir> <pcd_dir>" << std::endl;
        return 1;
    }
    std::string rosbag_dir = argv[1];
    std::string pcd_dir = argv[2];
    std::cout<< "pcd_dir:" << pcd_dir << std::endl;
    std::vector<std::vector<std::string>> groups = {
        {
        "edgar_classroom_run0", "edgar_classroom_run1", "edgar_classroom_run2", "edgar_classroom_run3","edgar_classroom_run4", "edgar_classroom_run5",
        }, {
        "ec_hallways_run0", "ec_hallways_run1", "ec_hallways_run2", "ec_hallways_run3", "ec_hallways_run4",
        }, {
        "arpg_lab_run0", "arpg_lab_run1", "arpg_lab_run2", "arpg_lab_run3", "arpg_lab_run4",
        }, {
        "outdoors_run0", "outdoors_run1", "outdoors_run2", "outdoors_run3", "outdoors_run4", "outdoors_run5", 
        "outdoors_run6", "outdoors_run7","outdoors_run8","outdoors_run9",
        },{
        "aspen_run0", "aspen_run1", "aspen_run2", "aspen_run3", "aspen_run4", "aspen_run5", "aspen_run6",
        "aspen_run7", "aspen_run8", "aspen_run9", "aspen_run10", "aspen_run11",
        },{
        "edgar_army_run0", "edgar_army_run1", "edgar_army_run2", "edgar_army_run3", "edgar_army_run4", "edgar_army_run5",
        },{
        "longboard_run0", "longboard_run1", "longboard_run2", "longboard_run3", "longboard_run4", "longboard_run5","longboard_run6", "longboard_run7", "longboard_run8",
        },   
    };

    if (has_pcd == false){
        convertRosbagToPcd(rosbag_dir, groups, pcd_dir);
        std::cout << "pcds saved" << std::endl;
    }

    alignAllSeqs(pcd_dir, groups);

    return 0;
}