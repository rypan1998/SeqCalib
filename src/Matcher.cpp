#include "Matcher.h"


/**
 * @brief 提取每组图像的 ArUco 角点坐标，并建立匹配关系
 * 
 * @param image_path 图像路径，%d/%04d.jpg or png 格式
 * @param group_num 图像组数
 * @param cam_num 相机数量
 * @param jpg2Cam 图像名称和相机 ID 的对应关系，比如 0000.jpg or png 对应 1 号相机
 * @param cam_start 相机 ID 的起始基准
 * @param group_start 图像组数的起始基准
 */
void Matcher::Match(const std::string &image_path, int group_num, int cam_num,
                    unordered_map<string, int> &jpg2Cam, int cam_start, int group_start) {
    // ! 根据标定板修改
    int markers_num(81);
    MatchData tmp_match_data(cam_num);
    m_match_data.resize(group_num * markers_num, tmp_match_data);

    // 建立角点亚像素化迭代规则
    TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 40, 0.001);

#pragma omp parallel for
    for (int group_id = group_start; group_id < group_start + group_num; ++group_id) {
        for (int cam_id = 0; cam_id < cam_num; ++cam_id) {
            // 1. 读图
            boost::format fmt(image_path);
            string image_name = (fmt % group_id % (cam_id + cam_start)).str();
            Mat img = imread(image_name, 0);

            // 2. 检测
            Ptr<aruco::Dictionary> dictionary =
                cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
            cv::Ptr<cv::aruco::CharucoBoard> board =
                cv::aruco::CharucoBoard::create(10, 10, 0.1f, 0.078f, dictionary); // ! 根据 ArUco 码的实际尺寸进行修改
            cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

            vector<int> aruco_ids;
            vector<vector<Point2f>> aruco_corners;
            aruco::detectMarkers(img, board->dictionary, aruco_corners, aruco_ids, params);
            // ! 如果是 OpenCV4 或更高版本，可能要改成下面这个写法
            // aruco::detectMarkers(img, board->getDictionary(), aruco_corners, aruco_ids, params);

            std::vector<cv::Point2f> marker_corners; // 角点 UV 坐标
            std::vector<int> marker_ids;
            if (aruco_ids.size() > 0) { // 检测到 ArUco
                cv::aruco::interpolateCornersCharuco(aruco_corners, aruco_ids, img, board,
                                                     marker_corners, marker_ids);
                if (marker_ids.size() > 0) { // 检测到 ArUco 的角点，一般是 4 个点
                    cornerSubPix(img, marker_corners, Size(5, 5), Size(-1, -1), criteria);

                    // 3. 保存结果
                    // char cam_char[50];
                    // sprintf(cam_char, "%04d.png", cam_id);
                    boost::format fmt("%04d.png"); // ! change this as you need !
                    int real_cam_id = jpg2Cam[(fmt % cam_id).str()];  // 查找真实图像的视角id
                    for (int cnt = 0; cnt < marker_ids.size(); ++cnt) {
                        m_match_data[group_id * markers_num + marker_ids[cnt]].FillData(
                            real_cam_id, marker_corners[cnt].x, marker_corners[cnt].y);
                    }
                }
            }
        }
    }

    // ! Log File
    ofstream fs("./log.txt");
    for (auto i : m_match_data) {
        for (auto j : i.pixel_points) {
            fs << "(" << j.first << ", " << j.second << ")"
            << " | ";
        }
        fs << endl;
    }
    fs.close();
}

void CreateIdMap(const std::string &db_path, std::unordered_map<std::string, int> &cam_id,
                 std::unordered_map<int, std::string> &cam_name) {
    // 1 存储文件名和id映射对
    sqlite3 *db;
    sqlite3_stmt *stmt = NULL;
    const char *z_tail;
    char *err_msg;
    int rc = sqlite3_open(db_path.c_str(), &db);
    if (rc != SQLITE_OK) {
        printf("error sqlite3_open\n");
        return;
    }
    if (sqlite3_prepare_v2(db, "SELECT image_id, name FROM images;", -1, &stmt, &z_tail) !=
        SQLITE_OK) {
        printf("error SELECT image_id, name FROM images\n");
        return;
    }
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        int image_id = sqlite3_column_int(stmt, 0);  //这里得到imageId 和 image_name_str的对应关系
        const unsigned char *image_name = sqlite3_column_text(stmt, 1);
        std::string image_name_str = std::string((char *)image_name);
        //保存每张图片的名称
        cam_id[image_name_str] = image_id - 1;
        cam_name[image_id] = image_name_str;
    }
    sqlite3_close(db);
    // debug
    for (const auto &element : cam_id) {
        printf("图片名 %s --- 映射ID %d\n", element.first.c_str(), element.second);
    }
}

void ExtractToDatabase(int num_cam, const std::string &db_path, const std::string &txt_path,
                       const std::vector<MatchData> &data,
                       std::unordered_map<int, std::string> &cam_name) {
    // 预处理Camera对象
    std::vector<Camera> cameras(num_cam, Camera(-1, num_cam));
    std::vector<int> point_ids(num_cam, 0);
    for (int i = 0; i < num_cam; ++i) {
        cameras[i].SetId(i + 1);
    }
    int num_all(data.size());  // 12*4
    std::cout << "num all: " << num_all << std::endl;
    for (int i = 0; i < num_all; ++i) {
        // 存储当前点在各个视图中的id
        std::vector<int> id(num_cam, -1);
        for (int cam_id = 0; cam_id < num_cam; ++cam_id) {
            // 当前视图不可见
            if (data[i].pixel_points[cam_id].first < 0) {
                continue;
            }
            std::pair<float, float> pixel_coord = data[i].pixel_points[cam_id];
            bool insert_ok = cameras[cam_id].AddKeypoints(pixel_coord, point_ids[cam_id]);
            if (insert_ok) {
                id[cam_id] = point_ids[cam_id];
                ++point_ids[cam_id];
            } else {
                // std::cout << cam_id << std::endl;
                id[cam_id] = cameras[cam_id].PointId(pixel_coord);
            }
        }
        for (int cam_id = 0; cam_id < num_cam; ++cam_id) {
            cameras[cam_id].AddMatches(id);
        }
        std::vector<int>().swap(id);
    }
    for (int i = 0; i < num_cam; ++i) {
        std::cout << "m_keypoints: " << i << " " << cameras[i].m_keypoints.size() << std::endl;
    }

    sqlite3 *db;
    sqlite3_stmt *stmt = NULL;
    const char *z_tail;
    char *err_msg;
    // 0 打开数据库文件
    int rc = sqlite3_open(db_path.c_str(), &db);
    if (rc != SQLITE_OK) {
        printf("error sqlite3_open\n");
        return;
    }
    // 2 删除原始keypoint记录
    if (sqlite3_prepare_v2(db, "DELETE FROM keypoints;", -1, &stmt, &z_tail) != SQLITE_OK) {
        printf("error DELETE FROM keypoints\n");
        return;
    }
    if (sqlite3_step(stmt) != SQLITE_DONE) {
        printf("error DELETE FROM keypoints\n");
        return;
    }
    // 3 删除原始matches记录
    if (sqlite3_prepare_v2(db, "DELETE FROM matches;", -1, &stmt, &z_tail) != SQLITE_OK) {
        printf("error DELETE FROM matches\n");
        return;
    }
    if (sqlite3_step(stmt) != SQLITE_DONE) {
        printf("error DELETE FROM matches\n");
        return;
    }
    // 4 删除原始two_view_geometries记录
    if (sqlite3_prepare_v2(db, "DELETE FROM two_view_geometries;", -1, &stmt, &z_tail) !=
        SQLITE_OK) {
        printf("error DELETE FROM two_view_geometries\n");
        return;
    }
    if (sqlite3_step(stmt) != SQLITE_DONE) {
        printf("error DELETE FROM two_view_geometries\n");
        return;
    }
    // 5 保存为 match.txt
    std::ofstream fs(txt_path, std::ios::out);
    if (!fs.is_open()) {
        printf("error open txt file\n");
        return;
    }
    for (int i = 0; i < num_cam; ++i) {
        int cam_id = cameras[i].GetId();
        int num_points = cameras[i].NumKeypoints();
        std::pair<float, float> *points_buffer = new std::pair<float, float>[num_points];
        // 拷贝点至blob buffer
        for (const auto &element : cameras[i].m_keypoints) {
            int id = element.second;
            points_buffer[id] = element.first;
        }
        // 5.1 写入keypoints
        char sql[255];
        sprintf(sql, "insert into keypoints values('%d','%d', '%d', ?);", cam_id, num_points, 2);
        sqlite3_prepare(db, sql, strlen(sql), &stmt, 0);
        {
            sqlite3_bind_blob(stmt, 1, points_buffer, num_points * sizeof(std::pair<float, float>),
                              nullptr);
            sqlite3_step(stmt);
        }
        sqlite3_finalize(stmt);
        // 5.2 写入matches
        for (int j = i + 1; j < num_cam; ++j) {
            uint32_t id_1 = cam_id;
            uint32_t id_2 = cameras[j].GetId();
            uint64_t pair_id = ImageIdsToPairId(id_1, id_2);
            int num_match = cameras[i].m_matches[j].size();
            std::pair<int, int> *matches_buffer = new std::pair<int, int>[num_match];
            memcpy(matches_buffer, &cameras[i].m_matches[j][0],
                   num_match * sizeof(std::pair<int, int>));
            // 写入db
            memset(sql, 255, 0);
            sprintf(sql, "insert into matches values('%ld','%d', '%d', ?);", pair_id, num_match, 2);
            sqlite3_prepare(db, sql, strlen(sql), &stmt, 0);
            {
                sqlite3_bind_blob(stmt, 1, matches_buffer, num_match * sizeof(std::pair<int, int>),
                                  nullptr);
                sqlite3_step(stmt);
            }
            sqlite3_finalize(stmt);
            delete[] matches_buffer;
            matches_buffer = nullptr;
            // 写入txt
            if (i != 0 || j != 1) {
                fs << std::endl;
            }
            fs << cam_name[id_1] << " " << cam_name[id_2] << std::endl;
            for (int k = 0; k < num_match; ++k) {
                fs << cameras[i].m_matches[j][k].first << " " << cameras[i].m_matches[j][k].second
                   << std::endl;
            }
        }
        delete[] points_buffer;
        points_buffer = nullptr;
    }
    sqlite3_close(db);
    fs.close();
}

/**
 * @brief 根据标定参数真值，生成符合实验要求的随机三维点
 * 
 * @param xmlPath 标定参数真值
 * @param cameraNumber 相机数量
 * @param maxPoints 生成三维点数量
 * @param boxSize 三维点的范围
 * @param trackRange 每个三维点的共视相机数
 * @param noise2D 二维检测误差
 * @param has_circle 自然特征分布：特征点在相机阵列之外的情况
 * @param is_track_exp 共视相机数量需要单独处理
 */
void Matcher::generateRandomPoints(const string &xmlPath, int cameraNumber, int maxPoints,
                                   vector<vector<int>> boxSize, vector<int> trackRange, int noise2D,
                                   bool has_circle, bool is_track_exp) {
    // 1. 读取标定参数的真值
    vector<Mat> Mat_P;
    for (int camID = 0; camID < cameraNumber; ++camID) {
        boost::format fmt(xmlPath);
        string path = (fmt % camID).str();
        FileStorage xmlFile = FileStorage(path, FileStorage::READ);
        Mat matrixP_4x4(4, 4, CV_64F);
        Mat matrixP_3x4(3, 4, CV_64F);
        xmlFile["P"] >> matrixP_4x4;

        matrixP_3x4 = matrixP_4x4.rowRange(0, 3).clone();
        matrixP_3x4.convertTo(matrixP_3x4, CV_64F);
        Mat_P.push_back(matrixP_3x4);
    }

    // 2. 随机生成三维点
    random_device rd;
    mt19937 mt(rd());
    std::default_random_engine generator(mt());

    // 当特征点在相机阵列之外时，仍要保证阵列之内仍有少部分特征点，否则会标定失败
    int rectanglePointsNum = has_circle ? 100 : maxPoints;
    
    for (int Points2DCount = 0; Points2DCount < maxPoints; ++Points2DCount) {
        while (true) {
            MatchData tmp_match_data(cameraNumber);  // 保存本轮三维点的结果
            vector<double> randomXYZ1; // 齐次坐标

            // 自然特征分布：
            // 先在相机阵列内产生 rectanglePointsNum 个点，这部分和其他实验情况通用；
            // 再把剩余点作为圆环夹层，放在相机阵列外围，进行模拟
            if (has_circle && Points2DCount >= rectanglePointsNum) {
                int R(40000), thickness(2000); // thickness 为圆环夹层的厚度
                double randomX, randomY, randomZ;
                // 随机坐标满足：R^2 <= x^2+y^2+z^2 <= (R+thickness)^2
                do {
                    std::uniform_real_distribution<double> dist_xy(-(R + thickness), R + thickness);
                    randomX = dist_xy(generator);
                    randomY = dist_xy(generator);
                } while (pow(randomX, 2) + pow(randomY, 2) < pow(R, 2) || pow(randomX, 2) + pow(randomY, 2) > pow(R + thickness, 2));

                std::uniform_real_distribution<double> dist_z(0, 2500); // 高度为 2.5m
                randomZ = dist_z(generator);

                randomXYZ1.push_back(randomX);
                randomXYZ1.push_back(randomY);
                randomXYZ1.push_back(randomZ);
                randomXYZ1.push_back(1.0);
            } else {
                // 生成在指定范围 box 内的三维点
                for (int axis = 0; axis < boxSize.size(); axis++) {
                    std::uniform_int_distribution<int> distribution(boxSize[axis][0],
                                                                    boxSize[axis][1]);
                    randomXYZ1.emplace_back(distribution(generator));
                }
                randomXYZ1.emplace_back(1.0);
            }

            // 转成矩阵表达方式，为反投影计算做准备
            Mat randomXYZ1Mat(4, 1, CV_64F);
            for (int i = 0; i < 4; i++) {
                randomXYZ1Mat.at<double>(i, 0) = randomXYZ1[i];
            }

            int valid_num = 0;
            for (int cam_id = 0; cam_id < cameraNumber; ++cam_id) {
                Mat random2DPointMat(3, 1, CV_64F);
                random2DPointMat = Mat_P[cam_id] * randomXYZ1Mat;

                // 需要除以最后一个元素，得到正确的 (u,v,1)
                random2DPointMat.at<double>(0, 0) /= random2DPointMat.at<double>(2, 0);
                random2DPointMat.at<double>(1, 0) /= random2DPointMat.at<double>(2, 0);
                random2DPointMat.at<double>(2, 0) = 1;

                // 加入 2D 检测噪声
                uniform_real_distribution<> distribution(noise2D - 1, noise2D);
                random2DPointMat.at<double>(0, 0) += distribution(generator);
                random2DPointMat.at<double>(1, 0) += distribution(generator);

                // 合法性校验：反投影得到的像素坐标点要在图像分辨率(1920x1080)范围内
                if (random2DPointMat.at<double>(0, 0) < 1920 &&
                    random2DPointMat.at<double>(1, 0) < 1080 &&
                    random2DPointMat.at<double>(0, 0) >= 0 &&
                    random2DPointMat.at<double>(1, 0) >= 0) {
                    // 临时保存结果
                    tmp_match_data.FillData(cam_id, random2DPointMat.at<double>(0, 0),
                                            random2DPointMat.at<double>(1, 0));
                    ++valid_num; // 反投影成功的相机数量，即共视数量
                } else {
                    tmp_match_data.FillData(cam_id, -1, -1);
                }
            }

            if (!is_track_exp && valid_num >= 2) { // 未进行共视关系实验：只要共视大于 2 即认为符合要求，要求太高的话很难满足
                m_match_data.emplace_back(tmp_match_data);
                break;
            }

            if (is_track_exp) {
                if (valid_num < trackRange[0]) {
                    continue;
                } else if (valid_num <= trackRange[1]) {
                    m_match_data.emplace_back(tmp_match_data);
                    break;
                } else { // 共视数量大于指定数，则随机选择视点改为(-1,-1)，取消在该视点的共视关系
                    vector<int> randomCamera; // 
                    uniform_int_distribution<int> distribution(trackRange[0], trackRange[1]);
                    int trackNumber = distribution(generator); // 指定共视关系数目

                    int randomCount = 0;
                    while (randomCount < cameraNumber - trackNumber) {
                        default_random_engine generator(mt()); // 为了防止随机数重复，重新指定因子
                        uniform_int_distribution<int> distribution(0, cameraNumber);
                        int randomCameraID = distribution(generator); 
                        
                        // 验证：是否能把 randomCameraID 相机置为 (-1,-1)
                        if (randomCamera.size() == 0) {
                            randomCamera.emplace_back(randomCameraID);
                            randomCount++;
                            continue;
                        } else {
                            auto result = find(randomCamera.begin(), randomCamera.end(), randomCameraID);
                            if (result == randomCamera.end()) { // 所选的 ID 未重复，可以放心加入
                                randomCamera.emplace_back(randomCameraID);
                                randomCount++;
                            } else { // 生成的 ID 重复！需要重新生成
                                continue;
                            }
                        }
                    }
                    for (auto id : randomCamera) {
                        tmp_match_data.FillData(id, -1, -1);
                    }
                    m_match_data.emplace_back(tmp_match_data);
                    break;
                }
            }
        }
    }

    // ! Log File
    // ofstream fs("./log.txt");
    // for (auto i : m_match_data) {
    //     for (auto j : i.pixel_points) {
    //         fs << "(" << j.first << ", " << j.second << ")" << " | ";
    //     }
    //     fs << endl;
    // }
}