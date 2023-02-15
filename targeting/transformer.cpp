#include <cmath>
#include <Eigen/Dense>
#include "pointlist.h"
#include "transformer.h"
#include "logger.h"

using namespace std;

// TODO: (CRITICAL) G_YAW and G_ROLL are SWAPPED!!! THIS IS AN ERROR!!! Recheck the math before just swapping the values in the function

// NOTE: This uses North, East, Down convention, and accepts angles in radians. Ensure values are in the correct format

// Returns a Matrix for translation? Ask Mo for info
Eigen::MatrixXd transformation_translation(double x, double y, double z){
    Eigen::MatrixXd t(4,4);
    t << 1, 0, 0, -x, 0, 1, 0, -y, 0, 0, 1, -z, 0, 0, 0, 1;
    return t;
}

// Returns a Matrix for rotation? Ask Mo for info
Eigen::MatrixXd transformation_rotation(double a, double b, double c){
    Eigen::MatrixXd t(4,4);
    t << cos(a)*cos(b), cos(a)*sin(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c), 0,
        sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c), 0,
        -sin(b), cos(b)*sin(c), cos(b)*cos(c), 0,
        0, 0, 0, 1;
    return t;
}

// More linear algebra. Ask Mo again :)
Eigen::MatrixXd intersection(Eigen::MatrixXd p0, Eigen::MatrixXd p1, Eigen::Vector3d p_co, Eigen::Vector3d p_no){
    double epsilon = 0.000001;
    Eigen::Vector3d u = p1 - p0;
    double dot = p_no.dot(u);
    if (abs(dot) > epsilon) {
        Eigen::Vector3d w = p0 - p_co;
        double fac = (-(p_no.dot(w)))/dot;
        u = u * fac;
        u = p0 + u;
        return u;
    } else {
        // Segment effectively parallel to plane uWu
        Eigen::MatrixXd x(1,3);
        // This might break something if ever we get a line parallel to the ground, hope it doesn't :)
        // TODO: Make sure that a coordinate of <0,0,0> is handled PROPERLY somewhere in the code
        // TODO: Maybe instead of returning it to main, within transform(...) we just fast return nothing and main can figure out what it means instead of wasting time building a struct for an invalid data point
        x << 0, 0, 0;
        return x;
    }

}


// Converts all inputs to determine the 3d location of the point
GPSPoint transform(Eigen::MatrixXd v_dist, double roll, double yaw, double pitch, double g_roll, double g_yaw, double g_pitch, Eigen::MatrixXd ccm, Eigen::MatrixXd ccm_inv, int pix_x, int pix_y, Eigen::MatrixXd g_dist, Eigen::MatrixXd c_dist, double f, Eigen::MatrixXd gnd, time_t timestamp){

    // Matrix used to find camera location coordinates
    Eigen::MatrixXd c(4,1);
    c << 0, 0, 0, 1;

    // Target coord in camera
    Eigen::MatrixXd pix(4,1);
    pix << pix_x, pix_y, 1, 1;

    // Roll matrix (90 deg around z-axis)
    Eigen::MatrixXd cc(4,4);
    cc << 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // Pitch matrix (90 deg around y-axis)
    Eigen::MatrixXd cam(4,4);
    cam << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 1;
    // Depth matrix (For depth of object in sensor frame w.r.t. camera frame)
    Eigen::MatrixXd im(4,4);
    im << f, 0, 0, 0, 0, f, 0, 0, 0, 0, f, 0, 0, 0, 0, 1;

    // Rotation and Translation from inertial frame to camera frame
    Eigen::MatrixXd trans_c = transformation_translation(c_dist(0,0), c_dist(0,1), c_dist(0,2));
    Eigen::MatrixXd rot_g = transformation_rotation(g_yaw, g_pitch, g_roll);
    Eigen::MatrixXd trans_g = transformation_translation(g_dist(0,0), g_dist(0,1), g_dist(0,2));
    Eigen::MatrixXd rot_v = transformation_rotation(yaw, pitch, roll);
    Eigen::MatrixXd trans_i = transformation_translation(v_dist(0,0), v_dist(0,1), v_dist(0,2));

    // Camera coordinates w.r.t. inertial frame
    Eigen::MatrixXd p_cc = trans_i.inverse() * rot_v * trans_g.inverse() * rot_g * trans_c.inverse() * c;
    // Coordinates of object seen in the camera sensor frame wrt inertial frame
    Eigen::MatrixXd q_obj = trans_i.inverse() * rot_v * trans_g.inverse() * rot_g * trans_c.inverse() * cam * im * cc * ccm_inv * pix;
    // Intersection point between line formed with the p_cc and q_obj and plane (ground), represents the location of the target w.r.t. the inertial frame
    Eigen::Vector3d p_cc_x(3);
    p_cc_x << p_cc(0,0), p_cc(1,0), p_cc(2,0);
    Eigen::Vector3d q_obj_x(3);
    q_obj_x << q_obj(0,0), q_obj(1,0), q_obj(2,0);

    // Find the point
    Eigen::Vector3d t = intersection(p_cc_x, q_obj_x, gnd.row(0), gnd.row(1));

    // Build struct
    GPSPoint point = {
            t,
            timestamp,
    };

    // TODO: Actually add a parameter in the struct containing how close to the normal and optical center we are (The closer the more accurate the prediction)
    return point;

}

std::string vec2string(Eigen::MatrixXd x){
    std::stringstream ss;
    ss << x;
    return ss.str();
}

GPSPoint transform_dummy(time_t timestamp){
    Eigen::MatrixXd ccm_inv(4,4);
    ccm_inv << 0.00204358, 0, -0.66121428, 0, 0, 0.00204224, -0.47667228, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    Eigen::MatrixXd ccm(3,3);
    ccm << 489.33767087, 0, 323.55705702, 0, 489.65953971, 233.40712684, 0, 0, 1;
    int pix_x = 323;
    int pix_y = 233;
    Eigen::MatrixXd v_dist(1,3);
    v_dist << 0, 0, -779;
    Eigen::MatrixXd g_dist(1,3);
    g_dist << 0, 0, 0;
    Eigen::MatrixXd c_dist(1,3);
    c_dist << 0, 0, 0;
    double yaw = 0;
    double pitch = (-M_PI/2); // (ask Mo why the -M_PI/2 is there if you wanna know)
    double roll = 0;
    double g_yaw = 0;
    double g_pitch = 0;
    double g_roll = 0;
    double f = 0.304;
    Eigen::MatrixXd gnd(2,3);
    gnd << 1, 1, 0, 0, 0, 1;
    return transform(v_dist, roll, yaw, pitch, g_roll, g_yaw, g_pitch, ccm, ccm_inv, pix_x, pix_y, g_dist, c_dist, f, gnd, timestamp);
}
