#include "slam/camera.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace slam {

Camera Camera::from_kitti_calib(const std::string& calib_file)
{
    // KITTI calib.txt format — each row is a label followed by 12 values.
    // P0: 3×4 projection matrix for left grayscale camera.
    // P0: fx 0 cx 0   0 fy cy 0   0 0 1 0
    //      [0]      [2]      [5]  [6]

    std::ifstream f(calib_file);
    if (!f.is_open()) {
        throw std::runtime_error("Cannot open calib file: " + calib_file);
    }

    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("P0:", 0) != 0) continue;  // skip until P0

        std::istringstream ss(line.substr(3));
        double vals[12];
        for (int i = 0; i < 12; ++i) ss >> vals[i];

        // P0 = K * [I | 0], so vals = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        Camera cam;
        cam.fx = vals[0];
        cam.fy = vals[5];
        cam.cx = vals[2];
        cam.cy = vals[6];
        cam.k1 = 0.0;
        cam.k2 = 0.0;
        // width/height unknown from calib alone; set when first image is loaded
        cam.width  = 0;
        cam.height = 0;
        return cam;
    }

    throw std::runtime_error("P0 not found in calib file: " + calib_file);
}

}  // namespace slam
