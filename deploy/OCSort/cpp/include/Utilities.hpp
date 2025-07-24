#ifndef OC_SORT_CPP_UTILITIES_HPP
#define OC_SORT_CPP_UTILITIES_HPP
#include "Eigen/Dense"


#include <random>
#include <chrono>
#include <cmath>
#include <algorithm> 
#include <opencv2/opencv.hpp>


namespace ocsort {
    /**
     * Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
    the aspect ratio
     * @param bbox
     * @return z
     */
    Eigen::VectorXf convert_bbox_to_z(Eigen::VectorXf bbox);
    Eigen::VectorXf speed_direction(Eigen::VectorXf bbox1, Eigen::VectorXf bbox2);
    Eigen::VectorXf convert_x_to_bbox(Eigen::VectorXf x);
    Eigen::VectorXf k_previous_obs(std::unordered_map<int, Eigen::VectorXf> observations_, int cur_age, int k);
    
    const double GOLDEN_RATIO = 0.618033988749895;
    std::array<int, 3> hsv_to_rgb_cpp(double h, double s, double v);
    std::array<int, 3> get_color(int idx, double s = 0.8, double vmin = 0.7);

}// namespace ocsort
#endif//OC_SORT_CPP_UTILITIES_HPP