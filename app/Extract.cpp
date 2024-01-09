#include <omp.h>
#include <string.h>

#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <popl.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "Matcher.h"

namespace po = boost::program_options;

int main(int argc, char *argv[]) {
    string image_path, project_path;
    int cam_num, group_num, cam_start, group_start;

    po::options_description desc("Allowed options");
    desc.add_options()("help,h", "produce help message")(
        "cam_num", po::value<int>(&cam_num), "camera numbers.")(
        "group_num", po::value<int>(&group_num), "group numbers.")(
        "cam_start", po::value<int>(&cam_start), "camera index start.")(
        "group_start", po::value<int>(&group_start), "group index start.")(
        "image_path", po::value<string>(&image_path), "image path, end with %04d.jpg")(
        "project_path", po::value<string>(&project_path), "colmap project directory path");

    bool is_aruco; // 使用随机三维点 or 进行 ArUco 检测
    int max_points, pixel_error;
    vector<int> track_length;
    vector<int> axis_range;  // 依次为 XYZ 三个轴的范围
    desc.add_options()("is_aruco", po::value<bool>(&is_aruco)->default_value(0), "if generate random 3D point.")(
        "max_points", po::value<int>(&max_points), "Numbers of random 3D points.")(
        "pixel_error", po::value<int>(&pixel_error), "2D detection pixel error")(
        "track_length", po::value<vector<int>>(&track_length)->multitoken(), "3D point track length range")(
        "axis_range", po::value<vector<int>>(&axis_range)->multitoken(), "3D point range");

    po::variables_map vm;
    po::store(po::parse_command_line(
                  argc, argv, desc,
                  po::command_line_style::unix_style ^ po::command_line_style::allow_short),
              vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << endl;
    }

    auto start_time = chrono::system_clock::now();

    std::unordered_map<std::string, int> id_map;
    std::unordered_map<int, std::string> name_map;

    Matcher *matcherObj = new Matcher;

    cout << "1. CreateIdMap.........." << endl;
    string database_path(project_path + "/database.db");
    CreateIdMap(database_path, id_map, name_map);

    if (is_aruco) {
        cout << "2. Match(ArUco)................" << endl;
        matcherObj->Match(image_path, group_num, cam_num, id_map, cam_start, group_start);
    } else {
        cout << "2. Match(Random Points)................" << endl;
        string xmlPath = "./xml_gt/%d.xml"; // 标定参数的真值
        vector<vector<int>> boxSize{{axis_range[0], axis_range[1]},
                                    {axis_range[2], axis_range[3]},
                                    {axis_range[4], axis_range[5]}};
        bool has_circle(0); // ! 自然特征分布实验
        bool is_track_exp(0); // ! 共视相机数量实验
        matcherObj->generateRandomPoints(xmlPath, cam_num, max_points, boxSize, track_length,
                                         pixel_error, has_circle, is_track_exp);
    }
    cout << "3. ExtractToDatabase...." << endl;
    string txt_path(project_path + "/match.txt");
    ExtractToDatabase(cam_num, database_path, txt_path, matcherObj->m_match_data, name_map);

    delete matcherObj;

    auto end_time = chrono::system_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end_time - start_time);
    cout << "程序运行时间："
         << double(duration.count()) * chrono::microseconds::period::num /
                chrono::microseconds::period::den
         << " 秒." << endl;

    return 0;
}