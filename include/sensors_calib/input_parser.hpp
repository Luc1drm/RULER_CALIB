/**
 * @file    input_parser.hpp
 *
 * @author  922
 *
 */


# pragma once

#include <iostream>
#include <vector>
#include <variant>

namespace calibparser
{
    enum class Options{
        OPT_CALIB_HANDLE_PATH = 0,
        OPT_INITGUESS_PATH = 1,
        OPT_IMG_PATH = 2,
        OPT_PCD_PATH = 3,
        OPT_CAMERA_INTRISINC = 4,
        OPT_OUTPUT_PATH = 5
    };

    class InputParser
    {
        public:
            InputParser(int argc, char **argv)
            : calib_handle(std::string("nofile")),
            initguess_path(std::string("nofile")),
            img_path(std::string("nofile")),
            pcd_path(std::string("nofile")),
            camera_info_path(std::string("nofile")),
            output_path(std::string("nofile"))
            {
                if((argc < 2)||
                (std::string(argv[1]) == "--help")||
                (std::string(argv[1]) == "--help")||
                (std::string(argv[1]) == "--help")||
                (std::string(argv[1]) == "--help")
                ){
                    print_help();
                    std::cout << "\nexit\n";
                    exit(EXIT_FAILURE);
                }

                for (int i = 1; i < argc; ++i)
                {   
                    
                    if (std::string(argv[i]) == "--calib_handler_path")
                    {
                        calib_handle = argv[i + 1];
                        i++;
                    }
                    else if (std::string(argv[i]) == "--initguess_path")
                    {
                        initguess_path = argv[i + 1];
                        i++;
                    }
                    else if (std::string(argv[i]) == "--img_path")
                    {
                        img_path = argv[i + 1];
                        i++;
                    }
                    else if (std::string(argv[i]) == "--pcd_path")
                    {
                        pcd_path = argv[i+1];
                        i++;
                    }
                    else if (std::string(argv[i]) == "--camera_info_path")
                    {
                        camera_info_path = argv[i+1];
                        i++; 
                    }
                    else if (std::string(argv[i]) == "--output_path")
                    {
                        output_path = argv[i+1];
                        i++;
                    }
                }
                print_input();
                check_input();
            }//InputParser

            void print_input()
            {
                std::cout<< "\n------------------------ Input Arguments --------------"
             << "--------------\n"
             << "calib_handler_path        : " << calib_handle         << "\n"
             << "initguess_path            : " << initguess_path       << "\n"
             << "img_path                  : " << img_path             << "\n"
             << "pcd_path                  : " << pcd_path             << "\n"
             << "camera_info_path          : " << camera_info_path     << "\n"
             << "output_path               : " << output_path          << "\n"
             << "\n";
            }

            void print_help()
            {
                std::cout << "\n------------------------------ Help -------------------"
                << "----------- \n"
                << "fisheye_calib                                           \\ \n"
                << "  --calib_handler_path      ${fisheye dir}/radar_info/calibration_handler_param.json                                  \\ \n"
                << "  --initguess_path          ${fisheye dir}/radar_info/initial_guess.json                        \\ \n"
                << "  --img_path                ${fisheye dir}/IMG.jpg                             \\ \n"
                << "  --pcd_path                ${fisheye dir}/scans.pcd               \\ \n"
                << "  --camera_info_path        ${fisheye dir}/radar info/camera info.json   \\ \n"
                << "  --output_path             ${fisheye dir}/out_put.json"
                << "\n";
            }

            void check_input()
            {
                if (calib_handle == "nofile")
                {
                    print_help();
                    std::cout << "\nexit\n";
                    exit(EXIT_FAILURE);
                }

                if (initguess_path == "nofile")
                {
                    print_help();
                    std::cout << "\nexit\n";
                    exit(EXIT_FAILURE);
                }

                if (img_path == "nofile")
                {
                    print_help();
                    std::cout << "\nexit\n";
                    exit(EXIT_FAILURE);
                }

                if (pcd_path == "nofile")
                {
                    print_help();
                    std::cout << "\nexit\n";
                    exit(EXIT_FAILURE);
                }

                if (camera_info_path == "nofile")
                {
                    print_help();
                    std::cout << "\nexit\n";
                    exit(EXIT_FAILURE);
                }

                if (output_path == "nofile")
                {
                    print_help();
                    std::cout << "\nexit\n";
                    exit(EXIT_FAILURE);
                } 
            }

            std::variant<bool, std::string> get(Options Opt) 
                {
                    switch(Opt)
                    {
                    case Options::OPT_CALIB_HANDLE_PATH       : return calib_handle;
                    case Options::OPT_INITGUESS_PATH          : return initguess_path;
                    case Options::OPT_IMG_PATH                : return img_path;
                    case Options::OPT_PCD_PATH                : return pcd_path;
                    case Options::OPT_CAMERA_INTRISINC        : return camera_info_path;
                    case Options::OPT_OUTPUT_PATH             : return output_path;
                    default: ;
                    }
                }

        private:
        std::string calib_handle;
        std::string initguess_path;
        std::string img_path;
        std::string pcd_path;
        std::string output_path;
        std::string camera_info_path;
        const std::string m_helper{"|Hint: Run with no arg or --h for help|"};
    };
}

// namespace POSparser
// {
//     enum class Options{
//         OPT_CALIB_HANDLE_PATH = 0,
//         OPT_INITGUESS_PATH = 1,
//         OPT_IMG_PATH = 2,
//         OPT_PCD_PATH = 3,
//         OPT_CAMERA_INTRISINC = 4,
//         OPT_OUTPUT_PATH = 5
//     };

//     class InputParser
//     {
//         public:
//             InputParser(int argc, char **argv)
//             : calib_handle(std::string("nofile")),
//             initguess_path(std::string("nofile")),
//             img_path(std::string("nofile")),
//             pcd_path(std::string("nofile")),
//             camera_info_path(std::string("nofile")),
//             output_path(std::string("nofile"))
//             {
//                 if((argc < 2)||
//                 (std::string(argv[1]) == "--help")||
//                 (std::string(argv[1]) == "--help")||
//                 (std::string(argv[1]) == "--help")||
//                 (std::string(argv[1]) == "--help")
//                 ){
//                     print_help();
//                     std::cout << "\nexit\n";
//                     exit(EXIT_FAILURE);
//                 }

//                 for (int i = 1; i < argc; ++i)
//                 {   
                    
//                     if (std::string(argv[i]) == "--calib_handler_path")
//                     {
//                         calib_handle = argv[i + 1];
//                         i++;
//                     }
//                     else if (std::string(argv[i]) == "--initguess_path")
//                     {
//                         initguess_path = argv[i + 1];
//                         i++;
//                     }
//                     else if (std::string(argv[i]) == "--img_path")
//                     {
//                         img_path = argv[i + 1];
//                         i++;
//                     }
//                     else if (std::string(argv[i]) == "--pcd_path")
//                     {
//                         pcd_path = argv[i+1];
//                         i++;
//                     }
//                     else if (std::string(argv[i]) == "--camera_info_path")
//                     {
//                         camera_info_path = argv[i+1];
//                         i++; 
//                     }
//                     else if (std::string(argv[i]) == "--output_path")
//                     {
//                         output_path = argv[i+1];
//                         i++;
//                     }
//                 }
//                 print_input();
//                 check_input();
//             }//InputParser

//             void print_input()
//             {
//                 std::cout<< "\n------------------------ Input Arguments --------------"
//              << "--------------\n"
//              << "calib_handler_path        : " << calib_handle         << "\n"
//              << "initguess_path            : " << initguess_path       << "\n"
//              << "img_path                  : " << img_path             << "\n"
//              << "pcd_path                  : " << pcd_path             << "\n"
//              << "camera_info_path          : " << camera_info_path     << "\n"
//              << "output_path               : " << output_path          << "\n"
//              << "\n";
//             }

//             void print_help()
//             {
//                 std::cout << "\n------------------------------ Help -------------------"
//                 << "----------- \n"
//                 << "fisheye_calib                                           \\ \n"
//                 << "  --calib_handler_path      ${fisheye dir}/radar_info/calibration_handler_param.json                                  \\ \n"
//                 << "  --initguess_path          ${fisheye dir}/radar_info/initial_guess.json                        \\ \n"
//                 << "  --img_path                ${fisheye dir}/IMG.jpg                             \\ \n"
//                 << "  --pcd_path                ${fisheye dir}/scans.pcd               \\ \n"
//                 << "  --camera_info_path        ${fisheye dir}/radar info/camera info.json   \\ \n"
//                 << "  --output_path             ${fisheye dir}/out_put.json"
//                 << "\n";
//             }

//             void check_input()
//             {
//                 if (calib_handle == "nofile")
//                 {
//                     print_help();
//                     std::cout << "\nexit\n";
//                     exit(EXIT_FAILURE);
//                 }

//                 if (initguess_path == "nofile")
//                 {
//                     print_help();
//                     std::cout << "\nexit\n";
//                     exit(EXIT_FAILURE);
//                 }

//                 if (img_path == "nofile")
//                 {
//                     print_help();
//                     std::cout << "\nexit\n";
//                     exit(EXIT_FAILURE);
//                 }

//                 if (pcd_path == "nofile")
//                 {
//                     print_help();
//                     std::cout << "\nexit\n";
//                     exit(EXIT_FAILURE);
//                 }

//                 if (camera_info_path == "nofile")
//                 {
//                     print_help();
//                     std::cout << "\nexit\n";
//                     exit(EXIT_FAILURE);
//                 }

//                 if (output_path == "nofile")
//                 {
//                     print_help();
//                     std::cout << "\nexit\n";
//                     exit(EXIT_FAILURE);
//                 } 
//             }

//             std::variant<bool, std::string> get(Options Opt) 
//                 {
//                     switch(Opt)
//                     {
//                     case Options::OPT_CALIB_HANDLE_PATH       : return calib_handle;
//                     case Options::OPT_INITGUESS_PATH          : return initguess_path;
//                     case Options::OPT_IMG_PATH                : return img_path;
//                     case Options::OPT_PCD_PATH                : return pcd_path;
//                     case Options::OPT_CAMERA_INTRISINC        : return camera_info_path;
//                     case Options::OPT_OUTPUT_PATH             : return output_path;
//                     default: ;
//                     }
//                 }

//         private:
//         std::string calib_handle;
//         std::string initguess_path;
//         std::string img_path;
//         std::string pcd_path;
//         std::string output_path;
//         std::string camera_info_path;
//         const std::string m_helper{"|Hint: Run with no arg or --h for help|"};
//     };
// }