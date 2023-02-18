#include <cmath>
#include <Eigen/Dense>
#include "pointlist.h"
#include "transformer.h"
#include "logger.h"

using namespace std;

// NOTE: This uses North, East, Down convention, and accepts angles in radians. Ensure values are in the correct format

// Creates translation transformation matrix
Eigen::MatrixXd transformation_translation(double x, double y, double z){
    Eigen::MatrixXd t(4,4);
    t << 1, 0, 0, -x, 0, 1, 0, -y, 0, 0, 1, -z, 0, 0, 0, 1;
    return t;
}

// Creates inversed translation transformation matrix
Eigen::MatrixXd transformation_inv_translation(double x, double y, double z){
    Eigen::MatrixXd t(4,4);
    t << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;
    return t;
}

// Creates rotation transformation matrix based on (yaw pitch roll) convention
Eigen::MatrixXd transformation_rotation(double a, double b, double c){
    Eigen::MatrixXd t(4,4);
    
    float cosPhi = cosf(c);
    float sinPhi = sinf(c);
    float cosThe = cosf(b);
    float sinThe = sinf(b);
    float cosPsi = cosf(a);
    float sinPsi = sinf(a);

    float dcm_0_0 = cosThe * cosPsi;
    float dcm_0_1 = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
    float dcm_0_2 = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

    float dcm_1_0 = cosThe * sinPsi;
    float dcm_1_1 = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
    float dcm_1_2 = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

    float dcm_2_0 = -sinThe;
    float dcm_2_1 = sinPhi * cosThe;
    float dcm_2_2 = cosPhi * cosThe;
    t << dcm_0_0, dcm_0_1, dcm_0_2, 0,
        dcm_1_0, dcm_1_1, dcm_1_2, 0,
        dcm_2_0, dcm_2_1, dcm_2_2, 0,
        0, 0, 0, 1;
    /*
    t << cos(a)*cos(b), sin(a)*cos(b), -sin(b), 0,
        sin(c)*sin(b)*cos(a)-cos(c)*sin(a), sin(c)*sin(b)*sin(a)+cos(c)*cos(a), sin(c)*cos(b), 0,
        cos(c)*sin(b)*cos(a)+sin(c)*sin(a), cos(c)*sin(b)*sin(a)-sin(c)*sin(a), cos(c)*cos(b), 0,
        0, 0, 0, 1;
    */
    return t;
}

// Creates rotation transformation matrix based on (yaw pitch roll) convention
// TODO: Double check this matrix
Eigen::MatrixXd transformation_inv_rotation(double a, double b, double c){
    Eigen::MatrixXd t(4,4);
    t << cos(a)*cos(b), cos(a)*sin(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c), 0,
        sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c), 0,
        -sin(b), cos(b)*sin(c), cos(b)*cos(c), 0,
        0, 0, 0, 1;
    return t;
}

// Finds the intersection between line (p0,p1) and plane (p_co,p_no)
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
        // This might break something if ever we get a line parallel to the ground, doubt it will though :)
        x << 0, 0, 0;
        return x;
    }

}


// Converts all inputs to determine the 3d location of the point
GPSPoint transform(Eigen::MatrixXd v_dist, double roll, double yaw, double pitch, double g_roll, double g_yaw, double g_pitch, Eigen::MatrixXd ccm, Eigen::MatrixXd ccm_inv, int pix_x, int pix_y, Eigen::MatrixXd g_dist, Eigen::MatrixXd c_dist, double f, Eigen::MatrixXd gnd, time_t timestamp){    
    // --------Position of object in camera frame--------
    
    /* Mo's method
    // Matrix used to find camera location coordinates
    Eigen::MatrixXd camera_in_f(4,1);
    camera_in_f << 0, 0, 0, 1;
    
    // Target coord in camera
    Eigen::MatrixXd pix(4,1);
    pix << pix_x, pix_y, 1, 1;

    // Depth matrix (For depth of object in sensor frame w.r.t. camera frame)
    Eigen::MatrixXd im(4,4);
    im << f, 0, 0, 0, 
        0, f, 0, 0, 
        0, 0, f, 0, 
        0, 0, 0, 1;

    // Roll matrix (90 deg around z-axis)
    Eigen::MatrixXd cam_z_rot(4,4);
    cam_z_rot << 0, -1, 0, 0, 
                1, 0, 0, 0, 
                0, 0, 1, 0, 
                0, 0, 0, 1;
    // Pitch matrix (90 deg around y-axis)
    Eigen::MatrixXd cam_y_rot(4,4);
    cam_y_rot << 0, 0, 1, 0, 
                0, 1, 0, 0, 
                -1, 0, 0, 0, 
                0, 0, 0, 1;
    
    // Calculate object in comera focal frame
    //Eigen::MatrixXd q_obj_in_f = cam_z_rot * cam_y_rot * ccm_inv * im * pix;
    // Old method - order of multiplication seems off -Zach
    Eigen::MatrixXd q_obj_in_f = cam_y_rot * im * cam_z_rot * ccm_inv * pix;
    //--------------------------------------------------------
    */
    
    // --------Rotation and Translation from inertial frame to camera frame--------
    // Translation from camera to focal center
    Eigen::MatrixXd trans_c_to_f = transformation_translation(c_dist(0,0), c_dist(0,1), c_dist(0,2));
    
    // Rotation from gimbal to camera 
    Eigen::MatrixXd rot_g_to_c = transformation_rotation(g_yaw, g_pitch, g_roll).inverse();
    
    // Translastion from body to gimbal rotation center
    Eigen::MatrixXd trans_b_to_g = transformation_translation(g_dist(0,0), g_dist(0,1), g_dist(0,2));
    
    // Rotation from vehicle to body
    Eigen::MatrixXd rot_v_to_b = transformation_rotation(yaw, pitch, roll).inverse();
    
    // Translation from inertial to vehicle
    Eigen::MatrixXd trans_i_to_v = transformation_translation(v_dist(0,0), v_dist(0,1), v_dist(0,2));
    
    // Transform from inertial to focal
    Eigen::MatrixXd T_i_to_f = trans_c_to_f * rot_g_to_c * trans_b_to_g * rot_v_to_b * trans_i_to_v;
    // Transform from focal to intertial
    Eigen::MatrixXd T_f_to_i = T_i_to_f.inverse();
    //----------------------------------------------------------------------------
    
    //----Coordinates of object seen in the camera sensor frame wrt inertial frame----
    // Focal center position in focal frame (0,0,0)
    Eigen::MatrixXd cam_pos_in_f(4,1);
    cam_pos_in_f << 0, 0, 0, 1;
    
    // Focal center position in inertial frame
    Eigen::MatrixXd focal_in_i = T_f_to_i*cam_pos_in_f;
    
    // z-axis unit vector in inertial frame
    Eigen::Vector4d k_hat;
    k_hat << 0, 0, 1, 0;
    
    // Unit vector from focal center to object in focal frame
    double f_px = 489.65953971;
    double ux = (double)pix_x - 320.0;
    double uy = (double)pix_y - 240.0;
    double l_hat_norm = sqrt(f_px*f_px + ux*ux + uy*uy);
    Eigen::Vector4d l_hat_f;
    l_hat_f << f_px/l_hat_norm, ux/l_hat_norm, uy/l_hat_norm, 1; // TODO: Actually get this value, currently assuming center of frame
    //std::cout << "l_hat_f:\n" << l_hat_f << "\n\n";
    //std::cout << "l_hat_norm:\n" << l_hat_norm << "\n\n";
    
    // Unit vector from focal to object in inertial frame
    Eigen::Vector4d l_hat_i = (rot_g_to_c * rot_v_to_b).inverse() * l_hat_f;
    //std::cout << "l_hat_i:\n" << l_hat_i << "\n\n";

    
    
    // Distance from focal center to object
    double L = -1*focal_in_i(2,0) / (k_hat.dot(l_hat_i));
    //std::cout << "L:\n" << L << "\n\n";
    
    // Position of object in inertial frame
    Eigen::MatrixXd q_obj_in_i = focal_in_i + L*l_hat_i;
    //--------------------------------------------------------------------------------
    
    /* Mo's method
    // Coordinates of object seen in the camera sensor frame wrt inertial frame
    Eigen::MatrixXd q_obj_in_i = trans_i_to_v.inverse() * rot_v_to_b.inverse() * trans_b_to_g.inverse() * rot_g_to_c.inverse() * trans_c_to_f.inverse() * q_obj_in_f;
    */
    
    /* Secondary validation method
    // Camera coordinates w.r.t. inertial frame
    Eigen::MatrixXd p_cc = trans_i_to_v.inverse() * rot_v_to_b.inverse() * trans_b_to_g.inverse() * rot_g_to_c.inverse() * trans_c_to_f.inverse() * c;
    
    // Intersection point between line formed with the p_cc and q_obj and plane (ground), represents the location of the target w.r.t. the inertial frame
    Eigen::Vector3d p_cc_x(3);
    p_cc_x << p_cc(0,0), p_cc(1,0), p_cc(2,0);
    Eigen::Vector3d q_obj_x(3);
    q_obj_x << q_obj(0,0), q_obj(1,0), q_obj(2,0);
    
    // Find the point
    Eigen::Vector3d t = intersection(p_cc_x, q_obj_x, gnd.row(0), gnd.row(1));
    */
    
    // Extract GPS point
    Eigen::Vector3d t(3);
    t << q_obj_in_i(0,0), q_obj_in_i(1,0), q_obj_in_i(2,0);
    
    // Build struct with GPS and timestamp
    GPSPoint point = {
            t,
            timestamp,
    };

    // TODO: Actually add a parameter in the struct containing how close to the normal and optical center we are (The closer the more accurate the prediction)
    return point;

}

// Converts vector to string
std::string vec2string(Eigen::MatrixXd x){
    std::stringstream ss;
    ss << x;
    return ss.str();
}

GPSPoint transform_dummy(){
    time_t timestamp;
    Eigen::MatrixXd ccm_inv(4,4);
    ccm_inv << 0.00204358, 0, -0.66121428, 0, 0, 0.00204224, -0.47667228, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    Eigen::MatrixXd ccm(3,3);
    ccm << 489.33767087, 0, 323.55705702, 0, 489.65953971, 233.40712684, 0, 0, 1;
    int pix_x = 320;
    int pix_y = 240;
    Eigen::MatrixXd v_dist(1,3);
    v_dist << 0, 0, -100;
    Eigen::MatrixXd g_dist(1,3);
    g_dist << 0, 0, 0;
    Eigen::MatrixXd c_dist(1,3);
    c_dist << 0, 0, 0;
    double yaw = 0;
    double pitch = 0; 
    double roll = 0;
    double g_yaw = M_PI/4;
    double g_pitch = -M_PI/4;
    double g_roll = 0;
    double f = 0.304;
    Eigen::MatrixXd gnd(2,3);
    gnd << 1, 1, 0, 0, 0, 1;
    return transform(v_dist, roll, yaw, pitch, g_roll, g_yaw, g_pitch, ccm, ccm_inv, pix_x, pix_y, g_dist, c_dist, f, gnd, timestamp);
}

/*
int main() {
    std::cout << transform_dummy().point << std::endl;
}
*/
