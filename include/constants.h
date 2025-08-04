#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath> // M_PIを使うため

constexpr int MAX_ITERATION = 35; // 最大反復回数 (ICPなどで使用)
constexpr double Leaning_rate = 0.5;
constexpr double EPS = 1e-7;

constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double DEG2RAD = M_PI / 180.0;

/*微小変位*/
constexpr double delta = 1.0e-8;

// 関数形式で使いたい場合
inline constexpr double rad2deg(double rad) { return rad * RAD2DEG; }
inline constexpr double deg2rad(double deg) { return deg * DEG2RAD; }

#endif // CONSTANTS_H

