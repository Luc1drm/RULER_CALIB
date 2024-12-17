/**
 * @file    SensorsCalibApp.cpp
 *
 * @author  btran
 *
 */

#include <iostream>

#include <pcl/io/pcd_io.h>

#include <sensors_calib/sensors_calib.hpp>
#include <sensors_calib/input_parser.hpp>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>

namespace
{
using PointCloudType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;
}  // namespace

void writeTransformToJson(const Eigen::Matrix4d& transform_matrix, const std::string& output_path) {
    // 创建 RapidJSON 文档
    rapidjson::Document document;
    document.SetObject();
    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

    // 添加矩阵元素到 JSON 中
    rapidjson::Value matrix(rapidjson::kArrayType);
    for (int i = 0; i < 4; ++i) {
        rapidjson::Value row(rapidjson::kArrayType);
        for (int j = 0; j < 4; ++j) {
            row.PushBack(transform_matrix(i, j), allocator);
        }
        matrix.PushBack(row, allocator);
    }
    
    // 添加矩阵到文档中
    document.AddMember("transform_matrix", matrix, allocator);

    // 将 JSON 文档写入文件
    std::ofstream ofs(output_path);
    rapidjson::OStreamWrapper osw(ofs);
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
    document.Accept(writer);
}


int main(int argc, char* argv[])
{   
    using calibparser::Options;
    //构造InputParser
    calibparser::InputParser Parser(argc, argv);

    std::string calib_handle        = std::get<std::string>(Parser.get(Options::OPT_CALIB_HANDLE_PATH    ));
    std::string initguess_path      = std::get<std::string>(Parser.get(Options::OPT_INITGUESS_PATH       ));
    std::string img_path            = std::get<std::string>(Parser.get(Options::OPT_IMG_PATH             ));
    std::string pcd_path            = std::get<std::string>(Parser.get(Options::OPT_PCD_PATH             ));
    std::string camera_info_path    = std::get<std::string>(Parser.get(Options::OPT_CAMERA_INTRISINC     ));
    std::string output_path         = std::get<std::string>(Parser.get(Options::OPT_OUTPUT_PATH          ));
    
    //从calib_handle静态获取有关参数
    const std::string PARAM_PATH = calib_handle;
    perception::CalibrationHandlerParam param = perception::getCalibrationHandlerParam(PARAM_PATH); 

    //获取路径等动态参数
    param.pathToInitialGuess = initguess_path;
    param.pathToImages       = img_path;
    param.pathToPointClouds  = pcd_path;
    param.pathToCameraInfo   = camera_info_path;

    perception::CalibrationHandler<PointCloudType>::Ptr calibrationHandler(new perception::CalibrationHandler<PointCloudType>(param));

    auto transform = calibrationHandler->optimize();
    const auto visualizedImgs = calibrationHandler->drawPointCloudOnImagePlane(transform);
    const auto projectedClouds = calibrationHandler->projectOnPointCloud(transform);

    printf("x: %f[m], y: %f[m], z: %f[m], r: %f[deg], p: %f[deg], y_deg: %f[deg]\n", transform(0), transform(1),
           transform(2), transform(3) * boost::math::double_constants::radian,
           transform(4) * boost::math::double_constants::radian, transform(5) * boost::math::double_constants::radian);

    double c_x = transform(0);
    double c_y = transform(1);
    double c_z = transform(2);
    double c_r = transform(3);
    double c_p = transform(4);
    double c_Y = transform(5);

    const std::string OUT_PATH = output_path;

    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.translation() << c_x, c_y, c_z;
    Eigen::Quaterniond quat(Eigen::AngleAxisd(c_r, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(c_p, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(c_Y, Eigen::Vector3d::UnitZ()));
    affine.linear() = quat.matrix();

    Eigen::Matrix4d t_mat = affine.matrix();

    writeTransformToJson(t_mat, OUT_PATH);

    for (std::size_t i = 0; i < visualizedImgs.size(); ++i) {
        const auto& curImg = visualizedImgs[i];
        cv::imwrite("img" + std::to_string(i) + ".png", curImg);
    }

    for (std::size_t i = 0; i < projectedClouds.size(); ++i) {
        const auto& curCloud = projectedClouds[i];
        pcl::io::savePCDFileBinary("cloud" + std::to_string(i) + ".pcd", *curCloud);
    }

    return EXIT_SUCCESS;
}
