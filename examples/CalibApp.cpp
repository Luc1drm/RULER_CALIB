#include <iostream>
#include <sensors_calib/sensors_calib.hpp>
#include <sensors_calib/input_parser.hpp>

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

//using TransformInfo = Eigen::Matrix<double, 6, 1>;
perception::TransformInfo CalibOptimize(int argc, char* argv[])
{   
    using calibparser::Options;
    //InputParser
    calibparser::InputParser Parser(argc, argv);
    
    // .\sensors_calib_app.exe  --calib_handler_path .\calibration_handler_param.json --initguess_path .\initial_guess_front.json 
    //--img_path .\front.jpg --pcd_path .\front.pcd --camera_info_path .\camera_info_front.json --output_path out_put.json

    std::string calib_handle        = std::get<std::string>(Parser.get(Options::OPT_CALIB_HANDLE_PATH    ));//静态参数 param.json
    std::string initguess_path      = std::get<std::string>(Parser.get(Options::OPT_INITGUESS_PATH       ));//初值
    std::string img_path            = std::get<std::string>(Parser.get(Options::OPT_IMG_PATH             ));//输入图像
    std::string pcd_path            = std::get<std::string>(Parser.get(Options::OPT_PCD_PATH             ));//输入点云
    std::string camera_info_path    = std::get<std::string>(Parser.get(Options::OPT_CAMERA_INTRISINC     ));//相机内参
    std::string output_path         = std::get<std::string>(Parser.get(Options::OPT_OUTPUT_PATH          ));//标定结果位姿文件输出路径

    perception::CalibrationHandlerParam param = perception::getCalibrationHandlerParam(calib_handle); 

    param.pathToInitialGuess = initguess_path;
    param.pathToImages       = img_path;
    param.pathToPointClouds  = pcd_path;
    param.pathToCameraInfo   = camera_info_path;

    perception::CalibrationHandler<PointCloudType>::Ptr calibrationHandler(new perception::CalibrationHandler<PointCloudType>(param));
    
    auto transform = calibrationHandler->optimize();

    //output
    printf("x: %f[m], y: %f[m], z: %f[m], r: %f[deg], p: %f[deg], y_deg: %f[deg]\n", transform(0), transform(1),
           transform(2), transform(3) * boost::math::double_constants::radian,
           transform(4) * boost::math::double_constants::radian, transform(5) * boost::math::double_constants::radian);

    //保存迭代结果至json文件
    //      output.json
    //     {
    //     "transform_matrix": [
    //         [
    //             -0.8569663566018391,
    //             0.05755052873971922,
    //             -0.5121490020436898,
    //             -0.062794
    //         ],
    //         ...
    //         [
    //             0.0,
    //             0.0,
    //             0.0,
    //             1.0
    //         ]
    //     ]
    //     }
    double c_x = transform(0);
    double c_y = transform(1);
    double c_z = transform(2);
    double c_R = transform(3);
    double c_P = transform(4);
    double c_Y = transform(5);

    const std::string OUT_PATH = output_path;

    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.translation() << c_x, c_y, c_z;
    Eigen::Quaterniond quat(Eigen::AngleAxisd(c_R, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(c_P, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(c_Y, Eigen::Vector3d::UnitZ()));
    affine.linear() = quat.matrix();

    Eigen::Matrix4d t_mat = affine.matrix();

    writeTransformToJson(t_mat, OUT_PATH);

    return transform;
}