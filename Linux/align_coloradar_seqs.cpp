// align coloradar sequences by brute force matching
#include <iostream>
#include <unordered_map>
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
            // if .pcd exist in output folder, then continue
            std::string firstpcdfilename = pcd_dir + "/" + seq + "/0.pcd";
            if (access(firstpcdfilename.c_str(), F_OK) != -1) {
                std::cout << "pcd files exist for " << seq << ", skip." << std::endl;
                continue;
            }
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
    const Eigen::Matrix4d &transform, std::string basepcd, std::string querypcd, std::string query_dir) {
    int g = -1;
    std::cout << "grouped_transforms.size:" << grouped_transforms.size() <<std::endl;
    for (auto &group : grouped_transforms) {
        std::pair<double, double> d = dist(group[0], transform);
        std::cout << "d: " << d.first << ", " << d.second << std::endl;
        if (d.first < 0.1 && d.second < 0.05) {
            // get index of group
            g = std::distance(&grouped_transforms[0], &group);
            std::string filename = query_dir + "/result/d_values.txt";
            ofstream outFile(filename, ios::app);
            outFile << "base:" << basepcd << " query:" << querypcd << " "
                    << "d: " << d.first << ", " << d.second << " g:" << g<< std::endl;
            outFile.close();
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
                           std::string basepcd, std::string querypcd, std::string query_dir)
{
    for (auto &group : grouped_transforms) {
        if (group.size() >50) {
            std::cout << "found correct transform" << std::endl;
            std::cout << "group[0]: \n" << group[0] << std::endl;
            std::string outputfile = query_dir + "/result/group0.txt";
            std::ofstream outFile(outputfile, ios::out);
            outFile << "base:" << basepcd << " query:" << querypcd << "\n" << group[0];
            outFile.close();

            return true;
        }
    }
    return false;
}

struct FrameCache {
    PointCloudPtr cloud;
    PointCloudPtr downsampled_cloud;
    vector<vector<float>> FPFH_descriptor;
};

void preparePcd(const std::string &rootdir, const std::string &fn, FrameCache &cache) {
    PointCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string pcd_path = rootdir + '/' + fn;
    pcl::io::loadPCDFile(pcd_path, *cloud);
    cache.cloud = cloud;

    float downsample = 0.6;
    PointCloudPtr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Voxel_grid_downsample(cloud, new_cloud, downsample);
    cache.downsampled_cloud = new_cloud;
    FPFH_descriptor(new_cloud, downsample * 5, cache.FPFH_descriptor);
}

float prepareCache(const std::vector<std::string> &pcdfiles, const std::string &rootdir, 
    std::unordered_map<std::string, FrameCache> &clouds) {
    float allresolution = 0;
    int i = 0;
    int N = std::min(10, (int)pcdfiles.size());
    for (const auto& fn : pcdfiles) {
        clouds[fn] = FrameCache();
        preparePcd(rootdir, fn, clouds[fn]);
        if (i < N) {
            float resolution = MeshResolution_mr_compute(clouds[fn].cloud);
            allresolution += resolution;
        }
        ++i;
    }
    return allresolution / N;
}

void alignTwoSeqs(const std::string &base_dir, const std::unordered_map<std::string, FrameCache> &base_clouds,
        const std::string &query_dir, float resolution) {
    std::cout << "aligning " << query_dir << " to " << base_dir << std::endl;
    std::vector<std::string> base_pcds = findPcds(base_dir);
    std::vector<std::string> query_pcds = findPcds(query_dir);

    srand(time(NULL));
    std::random_shuffle(query_pcds.begin(), query_pcds.end());
    std::random_shuffle(base_pcds.begin(), base_pcds.end());

    Eigen::AlignedVector<Eigen::AlignedVector<Eigen::Matrix4d>> grouped_transforms;
    Eigen::Matrix4d transform;

    ofstream outFile;
    std::string outputdir = query_dir + "/result";
    mkdir(outputdir.c_str(), 0777);
    std::string filename = query_dir + "/result/best_scores.txt";
    outFile.open(filename, ios::out);
    bool found = false;

    for (const auto& query_pcd : query_pcds) {
        FrameCache query_cache;
        preparePcd(query_dir, query_pcd, query_cache);
        PointCloudPtr query_cloud = query_cache.cloud;
        PointCloudPtr new_query_cloud = query_cache.downsampled_cloud;
        const vector<vector<float>> &query_feature = query_cache.FPFH_descriptor;
        std::cout << "query_pcd:" << query_pcd << std::endl;
        size_t bi = 0;
        double best_score = 0;
        for (const auto& base_pcd : base_pcds) {
            auto &base_cache = base_clouds.at(base_pcd);
            PointCloudPtr base_cloud = base_cache.cloud;
            PointCloudPtr new_base_cloud = base_cache.downsampled_cloud;
            const vector<vector<float>> &base_feature = base_cache.FPFH_descriptor;
            std::string base_pcd_path = base_dir + '/' + base_pcd;
            vector<Corre_3DMatch> correspondence;
            feature_matching(new_query_cloud, new_base_cloud, query_feature, base_feature, correspondence);
            std::cout<<"correspondence.size:"<< correspondence.size()<<"\n";

            vector<double> ov_lable;
            ov_lable.resize((int)correspondence.size());

            double score = 0;
            coloradar_registration(query_cloud, base_cloud, correspondence, ov_lable,
                                   transform, resolution, 0.99, score);
            outFile << base_pcd << " " << query_pcd << " " << score << endl;

            if (score > 30) {
                assignToClosestTransform(grouped_transforms, transform, base_pcd, query_pcd, query_dir);
            }
            found = foundCorrectTransform(grouped_transforms, base_pcd, query_pcd, query_dir);
            if (score > best_score) {
                best_score = score;
            }
            if (bi > base_pcds.size() / 5 && best_score < 30) {
                break;
            }
            correspondence.clear();
            ov_lable.clear();
            if (found) {
                std::cout << "found" <<std::endl;
                break;
            }
            ++bi;
        }
        if(found){
            break;
        }
    }
    outFile.close();
}


void alignAllSeqs(const std::string& pcd_dir, const std::vector<std::vector<std::string>> &groups) {
    std::cout << "alignAllSeqs" << std::endl;
    for (const auto &group : groups){
        int j = 0;
        std::unordered_map<std::string, FrameCache> base_clouds;
        std::string base_dir;
        float resolution;
        for (const auto &seq : group){
            if (j == 0) {
                base_dir = pcd_dir + "/" + group[0];
                std::vector<std::string> base_pcds = findPcds(base_dir);
                resolution = prepareCache(base_pcds, base_dir, base_clouds);
                ++j;
                continue;
            }

            std::string query_dir = pcd_dir + "/" + seq;
            alignTwoSeqs(base_dir, base_clouds, query_dir, resolution);
            ++j;
        }
    }
}

int main(int argc, char** argv) {
    std::cout << "Hello, World!" << std::endl;
    if (argc < 2) {
        std::cout << "Usage: align_coloradar_seqs <rosbag_dir> <pcd_dir>" << std::endl;
        return 1;
    }
    std::string rosbag_dir = argv[1];
    std::string pcd_dir = argv[2];
    std::cout<< "pcd_dir:" << pcd_dir << std::endl;
    add_overlap = false;
    low_inlieratio = false;
    no_logs = false;
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
        "longboard_run0", "longboard_run1", "longboard_run2", "longboard_run3", "longboard_run4", "longboard_run5","longboard_run6", "longboard_run7",
        },   
    };

    convertRosbagToPcd(rosbag_dir, groups, pcd_dir);
    std::cout << "pcds saved" << std::endl;

    alignAllSeqs(pcd_dir, groups);

    return 0;
}