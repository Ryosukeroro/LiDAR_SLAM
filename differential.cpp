#include "differential.h"
#include "constants.h" // de

double diffx(const Point& target, const Point& source) {

    double fx_delta = (target.x - (source.x + delta)) * (target.x - (source.x + delta)) 
                   + (target.y - source.y) * (target.y - source.y);
  double fx = (target.x - source.x) * (target.x - source.x) 
             + (target.y - source.y) * (target.y - source.y);
    return (fx_delta - fx) / delta;
}

/*double diffx(const Point& target, const Point& source) {
    Point s_plus = {source.x + delta, source.y};
    Point s_minus = {source.x - delta, source.y};

    double f_plus = 
}*/

double diffy(const Point& target, const Point& source) {
    Point s_plus = {source.x, source.y + delta};
    Point s_minus = {source.x, source.y - delta};

    double f_plus = (target.x - s_plus.x) * (target.x - s_plus.x) + (target.y - s_plus.y) * (target.y - s_plus.y);
    double f_minus = (target.x - s_minus.x) * (target.x - s_minus.x) + (target.y - s_minus.y) * (target.y - s_minus.y);

    return (f_plus - f_minus) / (2.0 * delta);
}

double difftheta(const Point& target, const Point& source) {
    // 微小な角度変化 (左右に)
    double theta_plus = delta;
    double theta_minus = -delta;

    // +delta 回転
    double cos_p = std::cos(theta_plus);
    double sin_p = std::sin(theta_plus);
    Point rotated_plus = {
        source.x * cos_p - source.y * sin_p,
        source.x * sin_p + source.y * cos_p
    };

    // -delta 回転
    double cos_m = std::cos(theta_minus);
    double sin_m = std::sin(theta_minus);
    Point rotated_minus = {
        source.x * cos_m - source.y * sin_m,
        source.x * sin_m + source.y * cos_m
    };

    // 誤差関数 = (target - rotated)^2
    double f_plus = (target.x - rotated_plus.x) * (target.x - rotated_plus.x)
      + (target.y - rotated_plus.y) * (target.y - rotated_plus.y);
    double f_minus = (target.x - rotated_minus.x) * (target.x - rotated_minus.x)
      + (target.y - rotated_minus.y) * (target.y - rotated_minus.y);

    return (f_plus - f_minus) / (2.0 * delta);
}