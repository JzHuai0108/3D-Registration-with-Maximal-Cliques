
#include "pc_utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/uniform_sampling.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>
#include "Eva.h"


typedef std::vector<std::pair<ros::Time, Eigen::Isometry3d>> PoseVector;

bool add_overlap;
bool low_inlieratio;
bool no_logs;

void loadTUMPoses(const std::string& filename, PoseVector& poses) {
    std::ifstream file(filename, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        std::string timestr;
        iss >> timestr;
        std::string secstr = timestr.substr(0, timestr.find('.'));
        std::string nsecstr = timestr.substr(timestr.find('.') + 1);
        if (nsecstr.size() < 9) {
            nsecstr.append(9 - nsecstr.size(), '0');
        }
        ros::Time stamp(std::stoul(secstr), std::stoul(nsecstr));

        Eigen::Vector3d t;
        Eigen::Quaterniond q;
        iss >> t.x() >> t.y() >> t.z() >> q.x() >> q.y() >> q.z() >> q.w();
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = t;
        pose.linear() = q.toRotationMatrix();
        poses.push_back(std::make_pair(stamp, pose));
    }

    file.close();
    std::cout << "Loaded " << poses.size() << " poses." << std::endl;
    std::cout << "First pose: " << poses.front().first << " " << poses.front().second.translation().transpose()
              << " " << Eigen::Quaterniond(poses.front().second.linear()).coeffs().transpose() << std::endl; 
    std::cout << "Last pose: " << poses.back().first << " " << poses.back().second.translation().transpose()
              << " " << Eigen::Quaterniond(poses.back().second.linear()).coeffs().transpose() << std::endl;
}

Eigen::Isometry3d interpolatePose(const ros::Time& t1, const Eigen::Isometry3d& p1,
        const ros::Time& t2, const Eigen::Isometry3d& p2, const ros::Time& t) {
    double alpha = (t - t1).toSec() / (t2 - t1).toSec();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = p1.translation() + alpha * (p2.translation() - p1.translation());
    Eigen::Quaterniond q1(p1.linear()), q2(p2.linear());
    pose.linear() = q1.slerp(alpha, q2).toRotationMatrix();
    return pose;
}

class Localizer {
private:
    PoseVector poses_;
    std::string tum_pose_file_;
    std::string tls_dir_;
    std::string output_dir_;
    TlsPositionVector tls_position_ids;
    std::string mac_file;
    std::ofstream mac_stream;
    std::string gicp_file;
    std::ofstream gicp_stream;

    pcl::KdTreeFLANN<pcl::PointXYZ> tls_kdtree;
    std::vector<int> loaded_tls_scanids;
    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_tls_scans;

    double meansquaredist_gicp = 0;
    size_t nummatches_gicp = 0;
    bool converged_gicp = false;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Localizer(const std::string& tum_pose_file, const std::string& tls_dir, const std::string& output_dir)
        : tum_pose_file_(tum_pose_file), tls_dir_(tls_dir), output_dir_(output_dir) {
        loadTUMPoses(tum_pose_file_, poses_);
        loadTlsScanPoses(tls_dir_);
        mac_file = output_dir_ + "/mac.txt";
        mac_stream.open(mac_file, std::ios::out);
        gicp_file = output_dir_ + "/gicp.txt";
        gicp_stream.open(gicp_file, std::ios::out);
    }

    ~Localizer() {
        mac_stream.close();
        gicp_stream.close();
    }

    void loadTlsScanPoses(const std::string &tls_dir) {
        std::cout << "TLS dir: " << tls_dir << std::endl;
        std::string tls_project_dir = tls_dir + "/project1/regis";
        load_tls_project_poses(tls_project_dir, tls_position_ids);
        tls_project_dir = tls_dir + "/project2/regis";
        load_tls_project_poses(tls_project_dir, tls_position_ids);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = tls_position_ids.size();
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);

        for (std::size_t i = 0; i < cloud->size (); ++i)
        {
            (*cloud)[i].x = tls_position_ids[i][0];
            (*cloud)[i].y = tls_position_ids[i][1];
            (*cloud)[i].z = tls_position_ids[i][2];
        }
        tls_kdtree.setInputCloud(cloud);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr toPointXYZ(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud, *cloud_xyz);
        return cloud_xyz;
    }

    Eigen::Isometry3d getLidarPose(const ros::Time& stamp) {
        auto it = std::lower_bound(poses_.begin(), poses_.end(), stamp, 
            [](const std::pair<ros::Time, Eigen::Isometry3d>& p, const ros::Time& t) {
                return p.first < t;
            });
        if (it == poses_.end()) {
            std::cerr << "Failed to find pose for timestamp: "
                    << stamp.sec << "." << std::setfill('0') << std::setw(9) << stamp.nsec << std::endl;
            return poses_.back().second;
        }
        if (std::fabs((it->first - stamp).toSec()) > 0.1) {
            std::cerr << "Too large time difference: found " << it->first.sec << "."
                    << std::setfill('0') << std::setw(9) << it->first.nsec << " for query " 
                    << stamp.sec << "." << std::setfill('0') << std::setw(9) << stamp.nsec << std::endl;
        }
        if (it == poses_.begin()) {
            return it->second;
        }
        return interpolatePose((it - 1)->first, (it - 1)->second, it->first, it->second, stamp);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadNearTlsScans(const Eigen::Isometry3d& tls_T_lidar) {
        Eigen::Vector3d pos = tls_T_lidar.translation();
        std::vector<int> pointIdxNKNSearch(2);
        std::vector<float> pointNKNSquaredDistance(2);
        if (tls_kdtree.nearestKSearch(pcl::PointXYZ(pos.x(), pos.y(), pos.z()), 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            int nearest_index = pointIdxNKNSearch[0];
            float first_dist = std::sqrt(pointNKNSquaredDistance[0]);
            float second_dist = std::sqrt(pointNKNSquaredDistance[1]);
            std::vector<int> nearest_scan_ids;
            if (first_dist / second_dist < 0.3) { // only load the first scan
                nearest_scan_ids.push_back(nearest_index);
            } else {
                nearest_scan_ids.push_back(nearest_index);
                nearest_scan_ids.push_back(pointIdxNKNSearch[1]);
            }
            if (nearest_scan_ids != loaded_tls_scanids) {
                loaded_tls_scanids = nearest_scan_ids;
                loaded_tls_scans = loadTlsScans(tls_dir_, tls_position_ids, nearest_scan_ids);
            }
            return loaded_tls_scans;
        } else {
            std::cerr << "Failed to find nearest TLS scan for lidar pose: " << pos.transpose() << std::endl;
            return nullptr;
        }
    }

    Eigen::Isometry3d registerLidarScanToTls(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr des_cloud,
            const float fpfh_radius_factor, bool verbose) {
        PointCloudPtr new_src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        PointCloudPtr new_des_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        float src_resolution = MeshResolution_mr_compute(src_cloud);
        float des_resolution = MeshResolution_mr_compute(des_cloud);
        float resolution = (src_resolution + des_resolution) / 2;

        float downsample = 5 * resolution;
        std::cout << "Average resolution: " << resolution << ", src resolution " << src_resolution
                    << ", dest resolution " << des_resolution << ". Downsample leaf size " << downsample << std::endl;
        Voxel_grid_downsample(src_cloud, new_src_cloud, downsample);
        Voxel_grid_downsample(des_cloud, new_des_cloud, downsample);
        std::cout << "After downsampling, src cloud size " << new_src_cloud->size()
                    << " dest cloud size " << new_des_cloud->size() << std::endl;
        std::cout << "Computing FPFH feature with factor " << fpfh_radius_factor
                    << " x resolution " << resolution << std::endl;
        vector<vector<float>> src_feature, des_feature;
        float fpfh_radius = fpfh_radius_factor * resolution;
        FPFH_descriptor(new_src_cloud, fpfh_radius, src_feature);
        FPFH_descriptor(new_des_cloud, fpfh_radius, des_feature);
        vector<Corre_3DMatch> correspondence;
        feature_matching(new_src_cloud, new_des_cloud, src_feature, des_feature,
                        correspondence);

        vector<double> ov_lable;
        ov_lable.resize((int)correspondence.size());

        cout << "Start registration with output path " << output_dir_ << endl;
        Eigen::Matrix4d Wt_T_Ws;
        registration(src_cloud, des_cloud, correspondence, ov_lable, output_dir_,
                    resolution, 0.99, Wt_T_Ws, verbose);
        Eigen::Isometry3d lidar_pose_mac = Eigen::Isometry3d::Identity();
        lidar_pose_mac.matrix() = Wt_T_Ws;
        cout << "Registration done!" << endl;
        // clear data
        src_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        des_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        new_src_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        new_des_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        src_feature.clear();
        src_feature.shrink_to_fit();
        des_feature.clear();
        des_feature.shrink_to_fit();
        correspondence.clear();
        correspondence.shrink_to_fit();
        ov_lable.clear();
        ov_lable.shrink_to_fit();
        return lidar_pose_mac;
    }

    Eigen::Isometry3d registerLidarScanToTlsGicp(pcl::PointCloud<pcl::PointXYZ>::ConstPtr src_cloud, 
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr des_cloud, const Eigen::Isometry3d &init_tls_T_lidar, 
            int num_gicp_iter) {
        PointCloudPtr new_src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        PointCloudPtr new_des_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        float leaf = 0.1; // leaf size in downsampling
        pcl::UniformSampling<pcl::PointXYZ> sor;
        sor.setRadiusSearch(leaf);
        sor.setInputCloud(src_cloud);
        sor.filter(*new_src_cloud);

        sor.setRadiusSearch(leaf);
        sor.setInputCloud(des_cloud);
        sor.filter(*new_des_cloud);

        GeneralizedIterativeClosestPointExposed<pcl::PointXYZ, pcl::PointXYZ> reg;
        reg.setInputSource(new_src_cloud);
        reg.setInputTarget(new_des_cloud);
        // use default parameters or set them yourself, for example:
        reg.setMaximumIterations(num_gicp_iter);
        // reg.setTransformationEpsilon(...);
        // reg.setRotationEpsilon(...);
        // reg.setCorrespondenceRandomness(...);
        // referring to https://github.com/PointCloudLibrary/pcl/issues/5180
        float maxCorrespondenceDist = 0.5; // max correspondence distance in icp
        float ransacTol = 0.05; // ransac outlier rejection threshold
        reg.setRANSACOutlierRejectionThreshold(ransacTol);
        reg.setMaxCorrespondenceDistance(maxCorrespondenceDist);

        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f tls_T_lidar = init_tls_T_lidar.matrix().cast<float>();
        reg.align(*output, tls_T_lidar);
        meansquaredist_gicp = reg.getFitnessScore(0.15);
        nummatches_gicp = reg.nr_;
        converged_gicp = reg.hasConverged();
        tls_T_lidar = reg.getFinalTransformation();
        Eigen::Isometry3d lidar_pose_gicp = Eigen::Isometry3d::Identity();
        lidar_pose_gicp.matrix() = tls_T_lidar.cast<double>();
        return lidar_pose_gicp;
    }

    void saveRegistrationResult(const ros::Time& lidar_stamp, const Eigen::Isometry3d& lidar_pose_mac,
            const Eigen::Isometry3d& lidar_pose_gicp) {
        Eigen::Quaterniond q(lidar_pose_mac.linear());
        Eigen::Vector3d t = lidar_pose_mac.translation();
        mac_stream << lidar_stamp.sec << "." << std::setfill('0') << std::setw(9) << lidar_stamp.nsec << " "
                   << std::fixed << std::setprecision(6) << t[0] << " " << t[1] << " " << t[2] << " "
                   << std::setprecision(9) << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
        for (size_t i = 0; i < loaded_tls_scanids.size(); ++i) {
            mac_stream << " " << loaded_tls_scanids[i];
        }
        mac_stream << std::endl;

        Eigen::Quaterniond q_gicp(lidar_pose_gicp.linear());
        Eigen::Vector3d t_gicp = lidar_pose_gicp.translation();
        gicp_stream << lidar_stamp.sec << "." << std::setfill('0') << std::setw(9) << lidar_stamp.nsec << " "
                    << std::fixed << std::setprecision(6) << t_gicp[0] << " " << t_gicp[1] << " " << t_gicp[2] << " "
                    << std::setprecision(9) << q_gicp.x() << " " << q_gicp.y() << " " << q_gicp.z() << " " << q_gicp.w()
                    << std::setprecision(4) << " " << meansquaredist_gicp << " " << nummatches_gicp << " " << converged_gicp << std::endl;
    }
};

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <undistorted.bag> <tum_pose.txt> <tls_dir> <output_dir>" << std::endl;
        return 1;
    }

    add_overlap = false;
    low_inlieratio = false;
    no_logs = false;

    rosbag::Bag bag(argv[1], rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("/hesai/pandar"));

    Localizer localizer(argv[2], argv[3], argv[4]);
    bool verbose = true;
    bool use_mac = true; // use max clique or pgo result to initialize gicp.
    int step = 399; // use a large step to check the result of max clique at different time points.
    // TODO(jhuai): We find that the max clique also fails miserably given pairs of lidar and TLS scans at random positions.
    int count = 1;
    for (const rosbag::MessageInstance& m : view) {
        if (count % step == 0) {
            std::cout << "Processing message " << count << std::endl;
        } else {
            count++;
            continue;
        }
        sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
        // TODO(jhuai): use a sliding window to accumulate lidar scans.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz = localizer.toPointXYZ(cloud);
        Eigen::Isometry3d tls_T_lidar_init = localizer.getLidarPose(cloud->header.stamp);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tls = localizer.loadNearTlsScans(tls_T_lidar_init);
        if (use_mac) {
            float fpfh_radius_factor = 100;
            tls_T_lidar_init = localizer.registerLidarScanToTls(cloud_xyz, cloud_tls, fpfh_radius_factor, verbose);
        }
        int num_gicp_iter = 10;
        Eigen::Isometry3d tls_T_lidar_gicp = localizer.registerLidarScanToTlsGicp(cloud_xyz, cloud_tls, tls_T_lidar_init, num_gicp_iter);
        localizer.saveRegistrationResult(cloud->header.stamp, tls_T_lidar_init, tls_T_lidar_gicp);
        count++;
    }
    bag.close();
}
