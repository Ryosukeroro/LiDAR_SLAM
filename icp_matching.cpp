#include "icp_matching.h"
#include "distance.h"
#include "constants.h"
#include "KD.h"
#include "path_visualizer.h"
#include <random>

// グローバル変数として現在の姿勢を定義
extern Pose current_pose; // 初期位置
extern std::vector<Point> target;


// void shuffle_data(std::vector<Point>& points) {
//     std::random_device rd;
//     std::mt19937 g(rd());
//     std::shuffle(points.begin(), points.end(), g);
// }

// std::vector<Point> createBatch(std::vector<Point>& source, size_t& m_current_offset, size_t m_batch_size) {
//     std::vector<Point> batch;
//     auto target_offset = m_current_offset + m_batch_size;

//     while (target_offset >= source.size()) {
//         while (m_current_offset < source.size()) {
//             batch.push_back(source[m_current_offset++]);
//         }
//         shuffle_data(source);
//         m_current_offset = 0;
//         target_offset = target_offset - source.size();
//     }
//     while (m_current_offset < target_offset) {
//         batch.push_back(source[m_current_offset++]);
//     }

//     return batch;
// }


// 回転 + 並進を適用する関数
// Point rotateAndTranslate(const Point& pt, double dx, double dy, double dtheta) {
//     double x_new = pt.x * std::cos(dtheta) - pt.y * std::sin(dtheta);
//     double y_new = pt.x * std::sin(dtheta) + pt.y * std::cos(dtheta);
//     return { static_cast<float>(x_new + dx), static_cast<float>(y_new + dy) };
// }

// 回転 + 並進を適用する関数
Point rotateAndTranslate(const Point& pt, double cx, double cy, double dx, double dy, double dtheta) {
    // 自己位置を基準に相対位置を計算
    double rel_x = pt.x - cx;
    double rel_y = pt.y - cy;

    // 回転 (中心:ロボットの位置)
    double rot_x = std::cos(dtheta) * rel_x - std::sin(dtheta) * rel_y;
    double rot_y = std::sin(dtheta) * rel_x + std::cos(dtheta) * rel_y;

    // 回転+並進
    return { 
        static_cast<double>(cx + rot_x + dx),
        static_cast<double>(cy + rot_y + dy)
         };
}

int findClosestPoint(const Point& point, const std::vector<Point>& target){
     if (target.empty()) {
        ROS_ERROR("findClosestPoint called with an empty target!");
        return -1; // エラー値を返す
    }
    int Index = -1;
    float minDist = std::numeric_limits<float>::max();
    for(size_t i = 0; i < target.size(); ++i){
        float dist = squaredDistance(target[i], point);
        if(dist < minDist){
            minDist = dist;
            Index = i;
        }
    }
    return Index;

}

//----------------------------------------
// ICPメイン処理
std::vector<Point> icp_scan_matching(const std::vector<Point>& source_points) {
    ROS_INFO("icp_scan_matching called! Source size: %lu", source_points.size());
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<Point> transformed = source_points;
    double previous_error = std::numeric_limits<double>::max();

    // 自己位置変化の蓄積 (current_pose にはまだ加えない)
    Pose tmp_pose = current_pose;
    Pose delta_pose = {0.0, 0.0, 0.0};
    // size_t m_batch_size = 30;
    // size_t m_current_offset = 0;
    // shuffle_data(transformed);

    for (int iter = 0; iter < MAX_ITERATION; ++iter) {
        double grad_dx = 0, grad_dy = 0, grad_theta = 0;
        double error_sum = 0.0;
        //auto batch = createBatch(transformed, m_current_offset, m_batch_size);
     //kdt::KDTree<Point> kdtree(target);
        for (const auto& src_pt : transformed ) {
            int idx = findClosestPoint(src_pt, target);
            //int idx = kdtree.nnSearch(src_pt);
            const Point& tgt_pt = target[idx];

            error_sum += squaredDistance(tgt_pt, src_pt);
            grad_dx += diffx(tgt_pt, src_pt);
            grad_dy += diffy(tgt_pt, src_pt);
            grad_theta += difftheta(tgt_pt, src_pt);
        }
        if (std::abs(previous_error - error_sum) < EPS) {
        ROS_INFO("ICP converged at iteration %d", iter);
        break;
        }

        int n = static_cast<int>(source_points.size());
        //int n =batch.size();
        double dx = -Leaning_rate * grad_dx / n;
        double dy = -Leaning_rate * grad_dy / n;
        double dtheta = -Leaning_rate * grad_theta / n;

        // // ロボットの自己位置更新
        // current_pose.x += dx;
        // current_pose.y += dy;
        // current_pose.theta += dtheta;

        

       

        // 点群の自己位置中心で回転・平行移動
        for (auto& pt : transformed) {
            pt = rotateAndTranslate(pt, tmp_pose.x, tmp_pose.y, dx, dy, dtheta);

        }
              // 蓄積
        delta_pose.x += dx;
        delta_pose.y += dy;
        delta_pose.theta += dtheta;
         // 仮の現在姿勢 (今回のステップまでの変位を加味した姿勢)
            tmp_pose.x = current_pose.x + delta_pose.x;
            tmp_pose.y = current_pose.y + delta_pose.y;
            tmp_pose.theta = current_pose.theta + delta_pose.theta;


        // if (std::abs(previous_error - error_sum) < EPS) {
        //     ROS_INFO("ICP converged at iteration %d", iter);
        //     break;
        // }

        previous_error = error_sum;


    }

    // 最後にロボットの自己位置に反映 (ICP全体も変位をまとめて)
    current_pose.x += delta_pose.x;
    current_pose.y += delta_pose.y;
    current_pose.theta += delta_pose.theta;
    

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    ROS_INFO("ICP completed in %ld ms", duration.count());

    return transformed;
}