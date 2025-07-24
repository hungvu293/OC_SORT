#include "../include/Utilities.hpp"
namespace ocsort {
    Eigen::VectorXf convert_bbox_to_z(Eigen::VectorXf bbox) {
        double w = bbox[2] - bbox[0];
        double h = bbox[3] - bbox[1];
        double x = bbox[0] + w / 2.0;
        double y = bbox[1] + h / 2.0;
        double s = w * h;
        double r = w / (h + 1e-6);
        Eigen::MatrixXf z(4, 1);
        z << x, y, s, r;
        return z;
    }
    Eigen::VectorXf speed_direction(Eigen::VectorXf bbox1, Eigen::VectorXf bbox2) {
        double cx1 = (bbox1[0] + bbox1[2]) / 2.0;
        double cy1 = (bbox1[1] + bbox1[3]) / 2.0;
        double cx2 = (bbox2[0] + bbox2[2]) / 2.0;
        double cy2 = (bbox2[1] + bbox2[3]) / 2.0;
        Eigen::VectorXf speed(2, 1);
        speed << cy2 - cy1, cx2 - cx1;
        double norm = sqrt(pow(cy2 - cy1, 2) + pow(cx2 - cx1, 2)) + 1e-6;
        return speed / norm;
    }
    Eigen::VectorXf convert_x_to_bbox(Eigen::VectorXf x) {
        float w = std::sqrt(x(2) * x(3));
        float h = x(2) / w;
        Eigen::VectorXf bbox = Eigen::VectorXf::Ones(4, 1);
        bbox << x(0) - w / 2, x(1) - h / 2, x(0) + w / 2, x(1) + h / 2;
        return bbox;
    }
    Eigen::VectorXf k_previous_obs(std::unordered_map<int, Eigen::VectorXf> observations_, int cur_age, int k) {
        if (observations_.size() == 0) return Eigen::VectorXf::Constant(5, -1.0);
        for (int i = 0; i < k; i++) {
            int dt = k - i;
            if (observations_.count(cur_age - dt) > 0) return observations_.at(cur_age - dt);
        }
        auto iter = std::max_element(observations_.begin(), observations_.end(), [](const std::pair<int, Eigen::VectorXf>& p1, const std::pair<int, Eigen::VectorXf>& p2) { return p1.first < p2.first; });
        int max_age = iter->first;
        return observations_[max_age];
    }

    std::array<int, 3> hsv_to_rgb_cpp(double h, double s, double v) {
        double r, g, b;

        if (s == 0.0) {
            r = v;
            g = v;
            b = v;
        } else {
            double i = std::floor(h * 6.0);
            double f = h * 6.0 - i;
            double p = v * (1.0 - s);
            double q = v * (1.0 - s * f);
            double t = v * (1.0 - s * (1.0 - f));

            switch (static_cast<int>(i)) {
                case 0: r = v; g = t; b = p; break;
                case 1: r = q; g = v; b = p; break;
                case 2: r = p; g = v; b = t; break;
                case 3: r = p; g = q; b = v; break;
                case 4: r = t; g = p; b = v; break;
                case 5: r = v; g = p; b = q; break;
                default: r = 0; g = 0; b = 0; break; 
            }
        }

        std::array<int, 3> rgb_color;
        rgb_color[0] = static_cast<int>(255 * r);
        rgb_color[1] = static_cast<int>(255 * g);
        rgb_color[2] = static_cast<int>(255 * b);

        rgb_color[0] = std::max(0, std::min(255, rgb_color[0]));
        rgb_color[1] = std::max(0, std::min(255, rgb_color[1]));
        rgb_color[2] = std::max(0, std::min(255, rgb_color[2]));

        return rgb_color;
    }


    std::array<int, 3> get_color(int idx, double s, double vmin) {
        double h = std::fmod(idx * GOLDEN_RATIO, 1.0);

        double v_range = 1.0 - vmin;
        double v_fmod_result = std::fmod(idx * GOLDEN_RATIO, v_range);
        double v = 1.0 - v_fmod_result;

        std::array<int, 3> rgb_color = hsv_to_rgb_cpp(h, s, v);

        std::array<int, 3> bgr_color;
        bgr_color[0] = rgb_color[2];
        bgr_color[1] = rgb_color[1];
        bgr_color[2] = rgb_color[0];

        return bgr_color;
    }

}// namespace ocsort