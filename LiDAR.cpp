#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "icp_matching.h"

#define RAD2DEG(x) ((x)*180./M_PI)

// struct Point {
//     float x;
//     float y;
// };

/*ロボットの自己位置*/
// struct Pose{
//     double x, y, theta;
// };

/*定義*/
#define MAX_iteration 30

/*収束判定の閾値*/
#define EPS  0.001

/*微小変位*/
#define delta 1.0e-7

/*学習率*/
#define learning_rate 1.2

std::vector<Point> target;

std::vector<Point> global_map;//グローバルマップを蓄積する

Pose current_pose = {0.0, 0.0, 0.0};  // ロボットの初期自己位置をグローバル変数として宣言

FILE* gnuplot_pipe = nullptr;

struct MapSnapshot {
     std::shared_ptr<const std::vector<Point>> map; // shared_ptrで受け取る
    Pose pose;
};

std::mutex snapshot_mutex;
std::condition_variable cv;
bool has_new_snapshot = false;
MapSnapshot latest_snapshot;
std::atomic<bool> running{true};

void plot(const std::vector<Point>& global_map_snapshot, const Pose& pose_snapshot) 

 {
    if (!gnuplot_pipe) return;

    fprintf(gnuplot_pipe, "set size ratio 1\n");
    fprintf(gnuplot_pipe, "set xrange [-10:10]\n");
    fprintf(gnuplot_pipe, "set yrange [-10:10]\n");
    
    // プロットのためのデータを送信
    fprintf(gnuplot_pipe, "plot '-' with points pointtype 7 pointsize 0.5 lc rgb 'black' title 'MAP', \
    '-' with vectors nohead lc rgb 'red' lw 2 title 'Robot Position', \
    '-' with vectors nohead lc rgb 'blue' lw 2 title 'Robot Direction'\n");

    

    // ファイルからの点群をプロット
    for (const auto& point : global_map_snapshot) {
        fprintf(gnuplot_pipe, "%f %f\n", point.x, point.y);
    }
    fprintf(gnuplot_pipe, "e\n");
 

       // ロボットの位置を示す十字マークを描画
    // 縦線
    fprintf(gnuplot_pipe, "%f %f %f %f\n", 
        current_pose.x, current_pose.y - 0.3,
        0.0, 0.6);  // y方向に±0.3の線
    fprintf(gnuplot_pipe, "e\n");
  
   // ロボットの向きを示す矢印を描画
    const double arrow_length = 0.5;  // 矢印の長さ
    fprintf(gnuplot_pipe, "%f %f %f %f\n",
        current_pose.x, current_pose.y,
        arrow_length * cos(current_pose.theta),
        arrow_length * sin(current_pose.theta));
    fprintf(gnuplot_pipe, "e\n");

    fprintf(gnuplot_pipe, "set label 1 'x: %.2f' at -9.5,9 left\n", current_pose.x);
    fprintf(gnuplot_pipe, "set label 2 'y: %.2f' at -9.5,8.5 left\n", current_pose.y);
    fprintf(gnuplot_pipe, "set label 3 'θ: %.2f°' at -9.5,8 left\n", RAD2DEG(current_pose.theta));


    fflush(gnuplot_pipe);
}

// プロット用スレッド
void plotThreadFunc() {
    while (running.load()) {
    std::unique_lock<std::mutex> lk(snapshot_mutex);
    cv.wait(lk, [] {return has_new_snapshot || !running.load();});
    if (!running.load()) break;

    // 最新スナップショットを取り出す (上書きバッファなのでmoveしたもよい)
    MapSnapshot snapshot = std::move(latest_snapshot);
    has_new_snapshot = false;
    lk.unlock();

    plot(*snapshot.map, snapshot.pose);

}}
//     std::vector<Point> points;
//     std::ifstream infile(file_path);
//     if (!infile.is_open()) {
//         ROS_ERROR("Failed to open file: %s", file_path.c_str());
//         return points;
//     }

//     std::string line;
//     while (std::getline(infile, line)) {
//         std::istringstream iss(line);
//         Point point;
//         if (iss >> point.x >> point.y) {
//             points.push_back(point);
//         }


// float distance(const Point& points, const Point& point){
//     return sqrt((points.x - point.x) * (points.x - point.x) + (points.y - point.y) * (points.y - point.y));

// }

// int findClosestPoint(const Point& point, const std::vector<Point>& target){
//     int Index = -1;
//     float minDist = std::numeric_limits<float>::max();
//     for(size_t i = 0; i < target.size(); ++i){
//         float dist = distance(target[i], point);
//         if(dist < minDist){
//             minDist = dist;
//             Index = i;
//         }
//     }
//     return Index;

// }

// float diffx(Point Target, Point SOurce){
//     float fx_delta = (Target.x - (SOurce.x + delta)) * (Target.x - (SOurce.x + delta)) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
//     float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
//     return (fx_delta - fx) / delta;
// }

// float diffy(Point Target, Point SOurce){
//     float fx_delta = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - (SOurce.y + delta)) * (Target.y - (SOurce.y + delta));
//     float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
//     return (fx_delta - fx) / delta;
// }

// float difftheta(Point Target, Point SOurce){
//     float fx_delta = (Target.x - ((SOurce.x)* cos(delta*M_PI/180)-(SOurce.y)* sin(delta*M_PI/180)))* (Target.x - ((SOurce.x)* cos(delta*M_PI/180)-(SOurce.y)* sin(delta*M_PI/180))) + (Target.y - ((SOurce.x) * (sin(delta*M_PI/180)) + (SOurce.y) * cos(delta*M_PI/180))) * (Target.y - ((SOurce.x) * (sin(delta*M_PI/180)) + (SOurce.y) * cos(delta*M_PI/180)));
//     float fx = (Target.x - SOurce.x) * (Target.x - SOurce.x) + (Target.y - SOurce.y) * (Target.y - SOurce.y);
//     return (fx_delta - fx) / delta;
// }



// std::vector<Point> icp_scan_matching( const std::vector<Point>& Source){//, const std::vector<Point>& target)
// auto start_time = std::chrono::high_resolution_clock::now();
//    std::vector<Point> transformed_source = Source;
//    float previous_error_sum = std::numeric_limits<float>::max(); // 前回の誤差を最大値で初期化
//    for(int iter = 0; iter <= MAX_iteration; ++iter){
//     std::vector<Point> target_closest;
//     float error_sum = 0;
//     double gradDx = 0;
//     double gradDy = 0;
//     double gradTheta = 0;
//     float dx = 0;
//     float dy = 0;
//     float dth = 0;
//     float dtheta = 0;
//     for(const auto& Source : transformed_source){
//          int index = findClosestPoint(Source,target);
//          target_closest.push_back(target[index]);
//          Point error = {target[index].x - Source.x, target[index].y - Source.y};
//          Point Target = {target[index].x, target[index].y};
//          Point SOurce = {Source.x, Source.y};
//          error_sum += error.x * error.x + error.y * error.y;
//          //std::cout << "error_sum: " << error_sum << std::endl;
//          gradDx += diffx(Target, SOurce);
//          gradDy += diffy(Target, SOurce);
//          gradTheta += difftheta(Target, SOurce);


//         //std::cout << "gradTheta: " << gradTheta << std::endl;
//     }
//     int num_points =Source.size(); // Sourceの点の数を取得

//     dx = (-gradDx / num_points) * learning_rate;
//     dy = (-gradDy / num_points) * learning_rate;
//     dth = (-gradTheta / num_points) * learning_rate;

//     //ロボットの自己位置を更新
//     current_pose.x += dx;
//     current_pose.y += dy;
//     current_pose.theta += dth;

//      //std::cout << "dy: " << dy << std::endl;

//      for(auto& Source : transformed_source){
//     //Source.x += dx;
//     float x_new = Source.x * cos(dth) - Source.y * sin(dth);
//     float y_new = Source.x * sin(dth) + Source.y * cos(dth);
//     Source.x = x_new + dx;
//     Source.y = y_new + dy;
// }

// /*収束条件のチェック*/
// if(std::abs(previous_error_sum - error_sum) < EPS){
// std::cout << "Converged after " << iter << "iterations." << std::endl;
// break;
// }
// previous_error_sum = error_sum;//前回の誤差を更新
//    }
//    //plot(transformed_source);
//     auto end_time = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//     std::cout << "ICP algorithm completed in " << duration.count() << " milliseconds." << std::endl;

Point transformPoint(const Point& point, const Pose& pose){
    Point transformed_point;
    //回転行列と平行移動を使ってロボット座標系から世界座標系に変換
    transformed_point.x = pose.x + point.x * cos(pose.theta) - point.y * sin(pose.theta);
    transformed_point.y = pose.y + point.x * sin(pose.theta) + point.y * cos(pose.theta);
    return transformed_point;
}

/*LiDARのスキャンデータ(点群)を蓄積する*/
void accumulateMap(const std::vector<Point>& scan_points){
    for(const auto& point : scan_points) {
        // ICP 各点をロボットの自己位置を使って世界座標に変換
        global_map.push_back(point);
    }
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->ranges.size();
        
    std::vector<Point> points;

    for (int i = 0; i < count; i++) {
        float angle = scan->angle_min + scan->angle_increment * i;
        float range = scan->ranges[i];

        if (range >= scan->range_min && range <= scan->range_max) {
            float x = range * cos(angle);
            float y = range * sin(angle);

            points.push_back({x, y});
            
        }
    }

    //スキャンデータをロボとの現在の位置に基づいて変換する
    std::vector<Point> transformed_points;
    for (const auto& point : points) {
        transformed_points.push_back(transformPoint(point, current_pose));
    }

    target = global_map;
    //最初のスキャンデータを使って初期の地図を作成
    if(global_map.empty()){
        global_map = transformed_points; //初期の地図として保存

    }else{
        //ICPによるスキャンデータのマッチングと地図の補正
        std::vector<Point> transformed_source = icp_scan_matching(transformed_points);

        //ICP決kの点群を世界座標に変換し、global_mapに蓄積
        accumulateMap(transformed_source);
    }

    // スナップショットを更新してプロットスレッドに通知 (コピーを取る)
    {
        std::lock_guard<std::mutex> lk(snapshot_mutex);
        // latest_snapshot.map = global_map; // コピーされる (必要なら間引きもここで)
        latest_snapshot.map = std::make_shared<const std::vector<Point>>(global_map);
        latest_snapshot.pose = current_pose;
        has_new_snapshot = true;
    }

    //plot(global_map);
    cv.notify_one();

}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "Lidar_node_client5");
    ros::NodeHandle n;
   


    gnuplot_pipe = popen("gnuplot -persist", "w");
    if (!gnuplot_pipe) {
        ROS_ERROR("Could not open pipe to gnuplot");
        return 1;
    }

    // プロットスレッド開始
    std::thread plot_thread(plotThreadFunc);

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);

    ros::spin();

    // シャットダウン処理
    running.store(false);
    cv.notify_one();
    if (plot_thread.joinable()) plot_thread.join();

     ROS_INFO("Node is shutting down. Saving map data to map.txt...");
   // ファイルを開く
     std::ofstream output_file("map.txt");
     if (output_file.is_open()) {
        // global_mapの全ての点をファイルに書き込む
        for (const auto& point : global_map) {
            output_file << point.x << " " << point.y << "\n";
        }
        output_file.close(); // ファイルを閉じる
            ROS_INFO("Map data successfully saved to map.txt");
    }  else {
        ROS_ERROR("Failed to open map.txt for writing.");
    }


    pclose(gnuplot_pipe);
    return 0;
}
