/**
 * @file    Utility.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <algorithm>
#include <limits>
#include <string>
#include <vector>
#include <unordered_set>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "../Types.hpp"
#include "PointCloudFilter.hpp"

namespace perception {

  // Eigen::Vector3i hash function
  struct Vector3iHash {
    std::size_t operator()(const Eigen::Vector3i &v) const {
      std::size_t seed = 0;
      for (int i = 0; i < 3; ++i) {
        seed ^= std::hash<int>()(v[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };

  inline cv::Scalar colorCodingReflectivityBGR(const int intensity) {
    std::uint8_t r, g, b;
    if (intensity < 30) {
      r = 0;
      g = static_cast<int>(intensity * 255 / 30) & 0xff;
      b = 255;
    } else if (intensity < 90) {
      r = 0;
      g = 0xff;
      b = static_cast<int>((90 - intensity) * 255 / 60) & 0xff;
    } else if (intensity < 150) {
      r = static_cast<int>((intensity - 90) * 255 / 60) & 0xff;
      g = 0xff;
      b = 0;
    } else {
      r = 0xff;
      g = static_cast<int>((255 - intensity) * 255 / (256 - 150)) & 0xff;
      b = 0;
    }

    return cv::Scalar(b, g, r);
  }

  TransformInfo getTransformInfo(const std::string transformationInfoPath);

  template<typename PointCloudType>
  cv::Point projectToImagePlane(const PointCloudType &point3d, const CameraInfo &cameraInfo) {
    // 获取相机内参矩阵和畸变系数
    const Eigen::Matrix3d kk = cameraInfo.K();
    const Eigen::Vector4d coeff = cameraInfo.coeffk();
    double fx = kk(0, 0);
    double fy = kk(1, 1);
    double cx = kk(0, 2);
    double cy = kk(1, 2);

    // 获取3D点的坐标
    long double X_ = point3d.x;
    long double Y_ = -point3d.y;
    long double Z_ = -point3d.z;
    if (fabs(Z_) < DBL_MIN) {
      Z_ = 1;  // 防止除以零
    }
    if (Z_ > -1e-3) {
      return {-1, -1};
    }

    long double r = sqrt((X_ * X_) / (Z_ * Z_) + (Y_ * Y_) / (Z_ * Z_));

    // 像素坐标的无畸变部分
    long double xu = -fx * X_ / Z_;
    long double yu = -fy * Y_ / Z_;

    long double theta = atan(r);
    if(abs(theta * boost::math::double_constants::radian) > 70.0){
      return {-1, -1};
    }
    
    // 使用畸变系数计算 theta_d
    double theta2 = theta * theta;
    double theta3 = theta2 * theta;
    double theta5 = theta2 * theta3;
    double theta7 = theta2 * theta5;
    double theta9 = theta2 * theta7;

    double theta_d = theta + coeff[0] * theta3 + coeff[1] * theta5 + coeff[2] * theta7 + coeff[3] * theta9;
    double inv_r = r > 1e-8 ? 1.0 / r : 1;
//        double cdist = r > 1e-8 ? theta_d * inv_r : 1;
    double cdist = theta_d * inv_r;

    // 计算最终的像素坐标
    long double u_ = cx + xu * cdist;
    long double v_ = cy - yu * cdist;

    // 返回整数类型的像素坐标
    return {static_cast<int>(u_), static_cast<int>(v_)};
  }


  template<typename PointCloudType>
  cv::Mat drawPointCloudOnImagePlane(const cv::Mat &img, const typename pcl::PointCloud<PointCloudType>::Ptr &inCloud,
                                     const CameraInfo &cameraInfo,
                                     const Eigen::Affine3d &affine = Eigen::Affine3d::Identity()) {
    typename pcl::PointCloud<PointCloudType>::Ptr alignedCloud(new pcl::PointCloud<PointCloudType>());
    pcl::transformPointCloud(*inCloud, *alignedCloud, affine.matrix());

    cv::Mat visualizedImg = img.clone();
    for (const auto &point: alignedCloud->points) {
      if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
        continue;
      }

      cv::Point imgPoint = projectToImagePlane<PointCloudType>(point, cameraInfo);
      if (imgPoint.x < 0 || imgPoint.x >= img.cols || imgPoint.y < 0 || imgPoint.y >= img.rows) {
        continue;
      }
      cv::circle(visualizedImg, imgPoint, 1 /* radius */, colorCodingReflectivityBGR(point.intensity), -1);
    }

    return visualizedImg;
  }

//   template<typename PointCloudType>
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr
//   projectOnPointCloud(const cv::Mat &img, const typename pcl::PointCloud<PointCloudType>::Ptr &inCloud,
//                       const CameraInfo &cameraInfo, const Eigen::Affine3d &affine = Eigen::Affine3d::Identity()) {
//     typename pcl::PointCloud<PointCloudType>::Ptr alignedCloud(new pcl::PointCloud<PointCloudType>());
//     pcl::transformPointCloud(*inCloud, *alignedCloud, affine.matrix());
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

//     // pair: index distance
//     std::vector<std::vector<std::pair<int, int>>> dists(img.rows,
//                                                         std::vector<std::pair<int, int>>(img.cols,
//                                                                                          std::make_pair(-1, 1e9)));
//     typename pcl::search::KdTree<PointCloudType>::Ptr kdtree(new pcl::search::KdTree<PointCloudType>());
//     kdtree->setInputCloud(alignedCloud);
//     std::vector<double> angles(alignedCloud->points.size(), 0.0);

//     for (std::size_t i = 0; i < inCloud->points.size(); ++i) {
//       if (i % 1000 == 0) {
//         std::cout << "Processing point: " << i << " of " << inCloud->points.size() << std::endl;
//       }
//       const auto &origPoint = inCloud->points[i];
//       const auto &alignedPoint = alignedCloud->points[i];
//       if (isnan(alignedPoint.x) || isnan(alignedPoint.y) || isnan(alignedPoint.z)) {
//         continue;
//       }
//       cv::Point imgPoint = projectToImagePlane<PointCloudType>(alignedPoint, cameraInfo);
//       if (imgPoint.x < 0 || imgPoint.x >= img.cols || imgPoint.y < 0 || imgPoint.y >= img.rows) {
//         continue;
//       }

//       double dist = std::sqrt(alignedCloud->points[i].x * alignedCloud->points[i].x +
//                               alignedCloud->points[i].y * alignedCloud->points[i].y +
//                               alignedCloud->points[i].z * alignedCloud->points[i].z);
//       if (dist > dists[imgPoint.y][imgPoint.x].second) {
//         continue;
//       }

//       // check normal vector
//       // knn search
//       int knn = 100;
//       std::vector<int> pointIdxNKNSearch(knn);
//       std::vector<float> pointNKNSquaredDistance(knn);
//       if (kdtree->nearestKSearch(alignedPoint, knn, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0) {
//         continue;
//       }
//       // Radius search
// //            if (kdtree->radiusSearch(alignedPoint, 0.1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0) {
// //                continue;
// //            }
// //            knn = pointIdxNKNSearch.size();
//       // calculate normal vector
//       Eigen::Vector3d mean = Eigen::Vector3d::Zero();
//       int valid_num = 0;
//       for (int j = 0; j < knn; ++j) {
//         const auto &point = alignedCloud->points[pointIdxNKNSearch[j]];
//         if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
//           continue;
//         }
//         valid_num++;
//         mean += Eigen::Vector3d(static_cast<double>(point.x), static_cast<double>(point.y),
//                                 static_cast<double>(point.z));
//       }
//       mean = mean / static_cast<double>(valid_num);
//       Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
//       for (int j = 0; j < knn; ++j) {
//         const auto &point = alignedCloud->points[pointIdxNKNSearch[j]];
//         if (isnan(point.x) || isnan(point.y) || isnan(point.z)) {
//           continue;
//         }
//         Eigen::Vector3d diff = Eigen::Vector3d(static_cast<double>(point.x), static_cast<double>(point.y),
//                                                static_cast<double>(point.z)) - mean;
//         cov += diff * diff.transpose();
//       }
//       cov = cov / static_cast<double>(valid_num);
//       Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
//       Eigen::Vector3d normal = eigensolver.eigenvectors().col(0);
//       // calculate angle
//       Eigen::Vector3d pointVec = Eigen::Vector3d(alignedPoint.x, alignedPoint.y, alignedPoint.z);
//       double angle = std::acos(normal.dot(pointVec) / (normal.norm() * pointVec.norm()));
//       double degAngle = angle * 180 / M_PI;
//       angles[i] = degAngle;
//       dists[imgPoint.y][imgPoint.x] = std::make_pair(i, dist);
//     }

//     for (int i = 0; i < img.rows; i++) {
//       for (int j = 0; j < img.cols; j++) {
//         if (dists[i][j].first == -1) {
//           continue;
//         }
//         const auto &origPoint = inCloud->points[dists[i][j].first];
//         const auto &alignedPoint = alignedCloud->points[dists[i][j].first];
//         pcl::PointXYZRGB outPoint;
//         outPoint.x = origPoint.x;
//         outPoint.y = origPoint.y;
//         outPoint.z = origPoint.z;
//         const auto &color = img.ptr<cv::Vec3b>(i)[j];
//         if (std::abs(angles[dists[i][j].first] - 90.0) < 20.0) {
//           outPoint.b = 0;
//           outPoint.g = 0;
//           outPoint.r = 0;
//         } else {
//           outPoint.b = color[0];
//           outPoint.g = color[1];
//           outPoint.r = color[2];
//         }
//         outCloud->points.emplace_back(outPoint);
//       }
//     }

//     outCloud->height = 1;
//     outCloud->width = outCloud->points.size();
//     return outCloud;
//   }


  template <typename PointCloudType>
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  projectOnPointCloud(const cv::Mat& img, const typename pcl::PointCloud<PointCloudType>::Ptr& inCloud,
                      const CameraInfo& cameraInfo, const Eigen::Affine3d& affine = Eigen::Affine3d::Identity())
  {
      typename pcl::PointCloud<PointCloudType>::Ptr alignedCloud(new pcl::PointCloud<PointCloudType>());
      pcl::transformPointCloud(*inCloud, *alignedCloud, affine.matrix());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

      for (std::size_t i = 0; i < inCloud->points.size(); ++i) {
          const auto& origPoint = inCloud->points[i];
          const auto& alignedPoint = alignedCloud->points[i];
          if (isnan(alignedPoint.x) || isnan(alignedPoint.y) || isnan(alignedPoint.z)) {
              continue;
          }
          cv::Point imgPoint = projectToImagePlane<PointCloudType>(alignedPoint, cameraInfo);
          if (imgPoint.x < 0 || imgPoint.x >= img.cols || imgPoint.y < 0 || imgPoint.y >= img.rows) {
              continue;
          }

          pcl::PointXYZRGB outPoint;
          outPoint.x = origPoint.x;
          outPoint.y = origPoint.y;
          outPoint.z = origPoint.z;
          const auto& color = img.ptr<cv::Vec3b>(imgPoint.y)[imgPoint.x];
          outPoint.b = color[0];
          outPoint.g = color[1];
          outPoint.r = color[2];
          outCloud->points.emplace_back(outPoint);
      }

      outCloud->height = 1;
      outCloud->width = outCloud->points.size();
      return outCloud;
  }

  inline std::vector<std::string> splitByDelim(const std::string &s, const char delimiter) {
    std::stringstream ss(s);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, delimiter)) {
      tokens.emplace_back(token);
    }
    return tokens;
  }

  inline std::vector<std::string> parseMetaDataFile(const std::string &metaDataFilePath) {
    std::ifstream inFile;
    inFile.open(metaDataFilePath);

    if (!inFile) {
      throw std::runtime_error("unable to open " + metaDataFilePath + "\n");
    }

    std::stringstream buffer;
    buffer << inFile.rdbuf();

    return splitByDelim(buffer.str(), '\n');
  }

  template<typename T>
  bool almostEquals(const T val, const T correctVal, const T epsilon = std::numeric_limits<T>::epsilon()) {
    const T maxXYOne = std::max({static_cast<T>(1.0f), std::fabs(val), std::fabs(correctVal)});
    return std::fabs(val - correctVal) <= epsilon * maxXYOne;
  }

  // get point's voxel index
  template<typename PointCloudType>
  Eigen::Vector3i getVoxelIdx(const PointCloudType &point, const double leafSize) {
    return Eigen::Vector3i(static_cast<int>(std::floor(point.x / leafSize)),
                           static_cast<int>(std::floor(point.y / leafSize)),
                           static_cast<int>(std::floor(point.z / leafSize)));
  }

  // to voxel grid
  template<typename PointCloudType>
  bool voxelGridFilter(const typename pcl::PointCloud<PointCloudType>::Ptr &inCloud,
                       const double leafSize,
                       std::unordered_map<Eigen::Vector3i, typename pcl::PointCloud<PointCloudType>::Ptr, Vector3iHash> *const voxelGrid) {
    if (leafSize <= 0) {
      return false;
    }
    if (!inCloud) {
      return false;
    }
    if (!voxelGrid) {
      return false;
    }
    for (const auto &point: inCloud->points) {
      Eigen::Vector3i voxelIdx = getVoxelIdx(point, leafSize);
      if (voxelGrid->find(voxelIdx) == voxelGrid->end()) {
        voxelGrid->emplace(voxelIdx,
                           typename pcl::PointCloud<PointCloudType>::Ptr(new pcl::PointCloud<PointCloudType>));
      }
      (*voxelGrid)[voxelIdx]->points.emplace_back(point);
    }

    const int minPointsThreshold = 5;

    // 检查每个体素中的点数
    for (auto it = voxelGrid->begin(); it != voxelGrid->end();) {
        if (it->second->points.size() < minPointsThreshold) {
            // 体素中的点数少于阈值，将其置为空
            it = voxelGrid->erase(it); // 删除体素
        } else {
            ++it; // 继续遍历
        }
    }

    return true;
  }

  // coloring point cloud
  template<typename PointCloudType>
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorPointCloud(const typename pcl::PointCloud<PointCloudType>::Ptr &inCloud,
                                                         const cv::Mat &img, const CameraInfo &cameraInfo,
                                                         const Eigen::Affine3d &affine = Eigen::Affine3d::Identity()) {
    typename pcl::PointCloud<PointCloudType>::Ptr alignedCloud(new pcl::PointCloud<PointCloudType>());
    //typename pcl::PointCloud<PointCloudType>::Ptr pointCloudPtr(new pcl::PointCloud<PointCloudType>());
    pcl::transformPointCloud(*inCloud, *alignedCloud, affine.matrix());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // visied map
    std::unordered_set<Eigen::Vector3i, Vector3iHash> visited;
    // voxel grid
    std::unordered_map<Eigen::Vector3i, typename pcl::PointCloud<PointCloudType>::Ptr, Vector3iHash> voxelGrid;
    const double leafSize = 0.1;
    const double step = leafSize / 2.0;
    voxelGridFilter<PointCloudType>(alignedCloud, leafSize, &voxelGrid);
    for (int i = 0; i < alignedCloud->points.size(); ++i) {
      if (i % 1000 == 0) {
        std::cout << "Processing point: " << i << " of " << alignedCloud->points.size() << std::endl;
      }
      const auto &point = alignedCloud->points[i];
      Eigen::Vector3i voxelIdx = getVoxelIdx(point, leafSize);

    if (voxelGrid.find(voxelIdx) == voxelGrid.end()) {
        //std::cout << "voxelIdx not found" << std::endl;
        continue;
    }


      cv::Point imgPoint = projectToImagePlane<PointCloudType>(point, cameraInfo);
      if (imgPoint.x < 0 || imgPoint.x >= img.cols || imgPoint.y < 0 || imgPoint.y >= img.rows) {
        continue;
      }
      const auto &color = img.ptr<cv::Vec3b>(imgPoint.y)[imgPoint.x];
      double len = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      Eigen::Vector3d pointVec = Eigen::Vector3d(point.x, point.y, point.z);
      double stepLen = len / step;
      Eigen::Vector3d basicVec = pointVec / stepLen;

      for (int j = static_cast<int>(1.0 / step); j < stepLen; ++j) {
        Eigen::Vector3d curVec = basicVec * j;
        Eigen::Vector3i curIdx = Eigen::Vector3i(static_cast<int>(std::floor(curVec[0] / leafSize)),
                                                 static_cast<int>(std::floor(curVec[1] / leafSize)),
                                                 static_cast<int>(std::floor(curVec[2] / leafSize)));

        if (voxelGrid.count(curIdx)) {
          if((stepLen - 1 - j) < 4) {
            pcl::PointXYZRGB outPoint{point.x, point.y, point.z, color[2], color[1], color[0]};
            outCloud->points.emplace_back(outPoint);
            break;
          }else{
            break;
          }
        }
      }
      visited.insert(voxelIdx);
    }

    outCloud->height = 1;
    outCloud->width = outCloud->points.size();
    pcl::transformPointCloud(*outCloud, *outCloud, affine.matrix().inverse().cast<float>());
    return outCloud;
  }

}  // namespace perception


#ifdef DEBUG
#define ENABLE_DEBUG 1
#include <iostream>
#else
#define ENABLE_DEBUG 0
#endif

#if ENABLE_DEBUG
#define DEBUG_LOG(...)                                                                                                 \
    {                                                                                                                  \
        char str[200];                                                                                                 \
        snprintf(str, sizeof(str), __VA_ARGS__);                                                                       \
        std::cout << "[" << __FILE__ << "][" << __FUNCTION__ << "][Line " << __LINE__ << "] >>> " << str << std::endl; \
    }
#else
#define DEBUG_LOG(...)
#endif
