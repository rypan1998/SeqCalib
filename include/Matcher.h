#ifndef _MATCHER_H_
#define _MATCHER_H_

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <set>
#include "HashFunc.h"
#include "Utilities.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <boost/format.hpp>
#include <random>

using namespace std;
using namespace cv;

struct MatchData 
{
    MatchData(int num) {
        pixel_points.resize(num, {-1.0f, -1.0f});
    }
    // 添加检测到的(u,v)坐标
    void FillData(int view_id, float u, float v) {
        pixel_points[view_id].first = u;
        pixel_points[view_id].second = v;
    }
    // pixel_points[i]表示当前点在视图i里的像素坐标
    std::vector<std::pair<float, float>> pixel_points;
};

struct MatcherBase {
    void CreateIdMap(const std::string& db_path);

    // 分组添加点 添加MatchData
    virtual void Match(const std::string &image_path, int group_num, int view_num, unordered_map<string, int>& jpg2Cam, int cam_start, int group_start) = 0;

    std::vector<MatchData> m_match_data;
    std::unordered_map<std::string, int> m_cam_id;
};

class Matcher : public MatcherBase
{
public:
    Matcher() {};

    std::vector<std::vector<std::string>> imageVector;

    void ReadImages(const std::string& image_path, int num_group, int num_view, int camera_name_start, int groupStart);
    
    void Match(const std::string &image_path, int group_num, int view_num, unordered_map<string, int>& jpg2Cam, int cam_start, int group_start);

    void generateRandomPoints(const string &xmlPath, int cameraNumber, int maxPoints,
                                   vector<vector<int>> boxSize, vector<int> trackRange, int noise2D,
                                   bool has_circle, bool is_track_exp);
                                   
    virtual ~Matcher() {};
};

struct Camera {
    typedef std::pair<float, float> pff;
    typedef std::pair<int, int> pii;

    int m_id;
    std::unordered_map<pff, int, HashFunc<pff>> m_keypoints;
    std::vector<std::vector<pii>> m_matches;

    Camera(int id, int num):m_id(id) { // 相机总数
        m_matches.resize(num);
    }

    void SetId(int id) {
        m_id = id;
    }

    const int GetId() const {
        return m_id;
    }

    const int PointId(const pff& point) {
        int result = m_keypoints[point];
        return result;
    }

    const int NumKeypoints() {
        return m_keypoints.size();
    }

    bool AddKeypoints(const pff& point, int id_expected) {
        if(m_keypoints.count(point)) {
            return false;
        }
        m_keypoints[point] = id_expected;
        return true;
    }

    void AddMatches(const std::vector<int>& ids) {
        int n(ids.size());
        int src_id = ids[m_id - 1];
        if(src_id < 0) {
            return;
        }
        for(int i = 0; i < n; ++i) {
            if(i == m_id - 1 || ids[i] == -1) {
                continue;
            }
            m_matches[i].push_back({src_id, ids[i]});
        }
    }

    
};

void ExtractToDatabase(int num_cam, const std::string& db_path, const std::string& txt_path, const std::vector<MatchData>& data, std::unordered_map<int, std::string>& cam_name);

void CreateIdMap(const std::string& db_path, std::unordered_map<std::string, int>& cam_id, std::unordered_map<int, std::string>& cam_name);

#endif