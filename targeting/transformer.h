#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <Eigen/Dense>
#include <cmath>
#include "pointlist.h"

using namespace std;

Eigen::MatrixXd transformation_translation(double x, double y, double z);
Eigen::MatrixXd transformation_rotation(double a, double b, double c);
Eigen::MatrixXd intersection(Eigen::MatrixXd p0, Eigen::MatrixXd p1, Eigen::Vector3d p_co, Eigen::Vector3d p_no);
GPSPoint transform(Eigen::MatrixXd v_dist, double roll, double yaw, double pitch, double g_roll, double g_yaw, double g_pitch, Eigen::MatrixXd ccm, Eigen::MatrixXd ccm_inv, int pix_x, int pix_y, Eigen::MatrixXd g_dist, Eigen::MatrixXd c_dist, double f, Eigen::MatrixXd gnd, time_t timestamp);
GPSPoint transform_dummy(time_t timestamp);

#endif
