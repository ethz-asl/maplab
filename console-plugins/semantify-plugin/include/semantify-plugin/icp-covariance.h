//////////////////////////////////////////////////////////////////////////////////////////////////
/// Reference: https://github.com/saimanoj18/3d-icp-covariance
#ifndef SEMANTIFY_PLUGIN_ICP_COVARIANCE_H_
#define SEMANTIFY_PLUGIN_ICP_COVARIANCE_H_

#include <iostream>
#include <math.h>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <Eigen/Dense>

namespace semantify {

void calculateICPCovariance(
    const Eigen::Matrix3Xd& data_pi, const Eigen::Matrix3Xd& model_qi,
    const Eigen::Matrix4d& transform, const std::vector<Eigen::Matrix3d> p_cov,
    const std::vector<Eigen::Matrix3d> q_cov,
    Eigen::Matrix<double, 6, 6>* ICP_COV) {
  CHECK_EQ(data_pi.cols(), model_qi.cols());
  CHECK_EQ(data_pi.cols(), p_cov.size());
  CHECK_EQ(data_pi.cols(), q_cov.size());

  double Tx = transform(0, 3);
  double Ty = transform(1, 3);
  double Tz = transform(2, 3);
  double roll = atan2(transform(2, 1), transform(2, 2));
  double pitch = asin(-transform(2, 0));
  double yaw = atan2(transform(1, 0), transform(0, 0));

  double x, y, z, a, b, c;
  x = Tx;
  y = Ty;
  z = Tz;
  a = yaw;
  b = pitch;
  c = roll;  // Important: According to the rotation matrix I used and after
             // verification, it is Yaw Pitch ROLL = [a,b,c]== [R] matrix used
             // in the MatLab also :)

  /* Flushing out in the form of XYZ ABC */
  // std::cout << "\nPrinting out [x, y, z, a, b, c] =  " <<x<<"    "<<y<<"
  // "<<z<<"    "<<a<<"    "<<b<<"    "<<c<<std::endl;

  // Matrix initialization
  Eigen::MatrixXd d2J_dX2(6, 6);
  d2J_dX2 = Eigen::MatrixXd::Zero(6, 6);

  /****  Calculating d2J_dX2  ****/
  for (int s = 0; s < data_pi.cols(); ++s) {
    double pix = data_pi(0, s);
    double piy = data_pi(1, s);
    double piz = data_pi(2, s);
    double qix = model_qi(0, s);
    double qiy = model_qi(1, s);
    double qiz = model_qi(2, s);

    /************************************************************
    d2J_dX2 -- X is the [R|T] in the form of [x, y, z, a, b, c]
    x, y, z is the translation part
    a, b, c is the rotation part in Euler format
    [x, y, z, a, b, c] is acquired from the Transformation Matrix returned by
    ICP.

    Now d2J_dX2 is a 6x6 matrix of the form

    d2J_dx2
    d2J_dxdy    d2J_dy2
    d2J_dxdz    d2J_dydz    d2J_dz2
    d2J_dxda    d2J_dyda    d2J_dzda   d2J_da2
    d2J_dxdb    d2J_dydb    d2J_dzdb   d2J_dadb   d2J_db2
    d2J_dxdc    d2J_dydc    d2J_dzdc   d2J_dadc   d2J_dbdc   d2J_dc2
    *************************************************************/

    double d2J_dx2, d2J_dydx, d2J_dzdx, d2J_dadx, d2J_dbdx, d2J_dcdx, d2J_dxdy,
        d2J_dy2, d2J_dzdy, d2J_dady, d2J_dbdy, d2J_dcdy, d2J_dxdz, d2J_dydz,
        d2J_dz2, d2J_dadz, d2J_dbdz, d2J_dcdz, d2J_dxda, d2J_dyda, d2J_dzda,
        d2J_da2, d2J_dbda, d2J_dcda, d2J_dxdb, d2J_dydb, d2J_dzdb, d2J_dadb,
        d2J_db2, d2J_dcdb, d2J_dxdc, d2J_dydc, d2J_dzdc, d2J_dadc, d2J_dbdc,
        d2J_dc2;

    // These terms are generated from the provided Matlab scipts. We just have
    // to copy the expressions from the matlab output with two very simple
    // changes. The first one being the the sqaure of a number 'a' is shown as
    // a^2 in matlab, which is converted to pow(a,2) in the below expressions.
    // The second change is to add ';' at the end of each expression :)
    // In this way, matlab can be used to generate these terms for various
    // objective functions of ICP and they can simply be copied to the C++ files
    // and with appropriate changes to ICP estimation, its covariance can be
    // easily estimated.

    d2J_dx2 = 2;
    d2J_dy2 = 2;
    d2J_dz2 = 2;
    d2J_dydx = 0;
    d2J_dxdy = 0;
    d2J_dzdx = 0;
    d2J_dxdz = 0;
    d2J_dydz = 0;
    d2J_dzdy = 0;
    d2J_da2 =
        (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
         piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
         pix * cos(a) * cos(b)) *
            (2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
             2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             2 * pix * cos(a) * cos(b)) -
        (2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
         2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         2 * pix * cos(b) * sin(a)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) +
        (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
         piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         pix * cos(b) * sin(a)) *
            (2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * pix * cos(b) * sin(a)) -
        (2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
         2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
         2 * pix * cos(a) * cos(b)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b));
    d2J_db2 =
        (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) *
            (2 * pix * cos(b) + 2 * piz * cos(c) * sin(b) +
             2 * piy * sin(b) * sin(c)) -
        (2 * piz * cos(b) * cos(c) - 2 * pix * sin(b) +
         2 * piy * cos(b) * sin(c)) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c)) -
        (2 * pix * cos(a) * cos(b) + 2 * piz * cos(a) * cos(c) * sin(b) +
         2 * piy * cos(a) * sin(b) * sin(c)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) +
        (piz * cos(a) * cos(b) * cos(c) - pix * cos(a) * sin(b) +
         piy * cos(a) * cos(b) * sin(c)) *
            (2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
             2 * piy * cos(a) * cos(b) * sin(c)) -
        (2 * pix * cos(b) * sin(a) + 2 * piz * cos(c) * sin(a) * sin(b) +
         2 * piy * sin(a) * sin(b) * sin(c)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) +
        (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
         piy * cos(b) * sin(a) * sin(c)) *
            (2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
             2 * piy * cos(b) * sin(a) * sin(c));
    d2J_dc2 =
        (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
         piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) *
            (2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) +
        (piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) *
            (2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) -
        (2 * piz * cos(b) * cos(c) + 2 * piy * cos(b) * sin(c)) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c)) +
        (2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) -
         2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b))) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) +
        (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)) *
            (2 * piy * cos(b) * cos(c) - 2 * piz * cos(b) * sin(c)) -
        (2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
         2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b))) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a));
    d2J_dxda = 2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) -
               2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
               2 * pix * cos(b) * sin(a);
    d2J_dadx = 2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) -
               2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
               2 * pix * cos(b) * sin(a);
    d2J_dyda = 2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
               2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
               2 * pix * cos(a) * cos(b);
    d2J_dady = 2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
               2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
               2 * pix * cos(a) * cos(b);
    d2J_dzda = 0;
    d2J_dadz = 0;
    d2J_dxdb = 2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
               2 * piy * cos(a) * cos(b) * sin(c);
    d2J_dbdx = 2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
               2 * piy * cos(a) * cos(b) * sin(c);
    d2J_dydb = 2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
               2 * piy * cos(b) * sin(a) * sin(c);
    d2J_dbdy = 2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
               2 * piy * cos(b) * sin(a) * sin(c);
    d2J_dzdb = -2 * pix * cos(b) - 2 * piz * cos(c) * sin(b) -
               2 * piy * sin(b) * sin(c);
    d2J_dbdz = -2 * pix * cos(b) - 2 * piz * cos(c) * sin(b) -
               2 * piy * sin(b) * sin(c);
    d2J_dxdc = 2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
               2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c));
    d2J_dcdx = 2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
               2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c));
    d2J_dydc = -2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) -
               2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c));
    d2J_dcdy = -2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) -
               2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c));
    d2J_dzdc = 2 * piy * cos(b) * cos(c) - 2 * piz * cos(b) * sin(c);
    d2J_dcdz = 2 * piy * cos(b) * cos(c) - 2 * piz * cos(b) * sin(c);
    d2J_dadb =
        (2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
         2 * piy * cos(b) * sin(a) * sin(c)) *
            (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
             piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             pix * cos(a) * cos(b)) -
        (2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
         2 * piy * cos(a) * cos(b) * sin(c)) *
            (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) +
        (2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
         2 * piy * cos(a) * cos(b) * sin(c)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) -
        (2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
         2 * piy * cos(b) * sin(a) * sin(c)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b));
    d2J_dbda =
        (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
         piy * cos(b) * sin(a) * sin(c)) *
            (2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
             2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             2 * pix * cos(a) * cos(b)) -
        (piz * cos(a) * cos(b) * cos(c) - pix * cos(a) * sin(b) +
         piy * cos(a) * cos(b) * sin(c)) *
            (2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * pix * cos(b) * sin(a)) +
        (2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
         2 * piy * cos(a) * cos(b) * sin(c)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) -
        (2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
         2 * piy * cos(b) * sin(a) * sin(c)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b));
    d2J_dbdc =
        (2 * piy * cos(a) * cos(b) * cos(c) -
         2 * piz * cos(a) * cos(b) * sin(c)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) +
        (2 * piy * cos(b) * cos(c) * sin(a) -
         2 * piz * cos(b) * sin(a) * sin(c)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) -
        (2 * piy * cos(b) * cos(c) - 2 * piz * cos(b) * sin(c)) *
            (pix * cos(b) + piz * cos(c) * sin(b) + piy * sin(b) * sin(c)) -
        (2 * piy * cos(c) * sin(b) - 2 * piz * sin(b) * sin(c)) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c)) +
        (2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
         2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) *
            (piz * cos(a) * cos(b) * cos(c) - pix * cos(a) * sin(b) +
             piy * cos(a) * cos(b) * sin(c)) -
        (2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) *
            (piz * cos(b) * cos(c) * sin(a) - pix * sin(a) * sin(b) +
             piy * cos(b) * sin(a) * sin(c));
    d2J_dcdb =
        (2 * piy * cos(a) * cos(b) * cos(c) -
         2 * piz * cos(a) * cos(b) * sin(c)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) +
        (2 * piy * cos(b) * cos(c) * sin(a) -
         2 * piz * cos(b) * sin(a) * sin(c)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) -
        (piy * cos(b) * cos(c) - piz * cos(b) * sin(c)) *
            (2 * pix * cos(b) + 2 * piz * cos(c) * sin(b) +
             2 * piy * sin(b) * sin(c)) -
        (2 * piy * cos(c) * sin(b) - 2 * piz * sin(b) * sin(c)) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c)) +
        (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
         piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) *
            (2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
             2 * piy * cos(a) * cos(b) * sin(c)) -
        (piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) *
            (2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
             2 * piy * cos(b) * sin(a) * sin(c));
    d2J_dcda =
        (2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) -
        (piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
         piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) *
            (2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * pix * cos(b) * sin(a)) -
        (piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) *
            (2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
             2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             2 * pix * cos(a) * cos(b)) +
        (2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
         2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a));
    d2J_dadc =
        (2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) -
        (2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
         2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) *
            (piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) -
        (2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
         2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) *
            (piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
             piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             pix * cos(a) * cos(b)) +
        (2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
         2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a));
    Eigen::MatrixXd d2J_dX2_temp(6, 6);
    d2J_dX2_temp << d2J_dx2, d2J_dydx, d2J_dzdx, d2J_dadx, d2J_dbdx, d2J_dcdx,
        d2J_dxdy, d2J_dy2, d2J_dzdy, d2J_dady, d2J_dbdy, d2J_dcdy, d2J_dxdz,
        d2J_dydz, d2J_dz2, d2J_dadz, d2J_dbdz, d2J_dcdz, d2J_dxda, d2J_dyda,
        d2J_dzda, d2J_da2, d2J_dbda, d2J_dcda, d2J_dxdb, d2J_dydb, d2J_dzdb,
        d2J_dadb, d2J_db2, d2J_dcdb, d2J_dxdc, d2J_dydc, d2J_dzdc, d2J_dadc,
        d2J_dbdc, d2J_dc2;
    d2J_dX2 = d2J_dX2 + d2J_dX2_temp;

  }  // End of the FOR loop!!!

  // std::cout << "\n**************\n Successfully Computed d2J_dX2
  // \n**************\n" << std::endl;

  // Now its time to calculate d2J_dZdX , where Z are the measurements Pi and
  // Qi, X = [x,y,z,a,b,c]

  // n is the number of correspondences
  int n = data_pi.cols();

  /*  Here we check if the number of correspondences between the source and the
   target point clouds are greater than 200. if yes, we only take the first 200
   correspondences to calculate the covariance matrix You can try increasing it
   but if its too high, the system may run out of memory and give an exception
   saying

   terminate called after throwing an instance of 'std::bad_alloc'
   what():  std::bad_alloc
   Aborted (core dumped)

*/

  // std::cout << "\nNumber of Correspondences used for ICP's covariance
  // estimation = " << n << std::endl;

  Eigen::MatrixXd d2J_dZdX(6, 6 * n);

  for (int k = 0; k < n; ++k)  // row
  {
    // here the current correspondences are loaded into Pi and Qi
    double pix = data_pi(0, k);
    double piy = data_pi(1, k);
    double piz = data_pi(2, k);
    double qix = model_qi(0, k);
    double qiy = model_qi(1, k);
    double qiz = model_qi(2, k);

    Eigen::MatrixXd d2J_dZdX_temp(6, 6);

    double d2J_dpix_dx, d2J_dpiy_dx, d2J_dpiz_dx, d2J_dqix_dx, d2J_dqiy_dx,
        d2J_dqiz_dx, d2J_dpix_dy, d2J_dpiy_dy, d2J_dpiz_dy, d2J_dqix_dy,
        d2J_dqiy_dy, d2J_dqiz_dy, d2J_dpix_dz, d2J_dpiy_dz, d2J_dpiz_dz,
        d2J_dqix_dz, d2J_dqiy_dz, d2J_dqiz_dz, d2J_dpix_da, d2J_dpiy_da,
        d2J_dpiz_da, d2J_dqix_da, d2J_dqiy_da, d2J_dqiz_da, d2J_dpix_db,
        d2J_dpiy_db, d2J_dpiz_db, d2J_dqix_db, d2J_dqiy_db, d2J_dqiz_db,
        d2J_dpix_dc, d2J_dpiy_dc, d2J_dpiz_dc, d2J_dqix_dc, d2J_dqiy_dc,
        d2J_dqiz_dc;

    d2J_dpix_dx = 2 * cos(a) * cos(b);
    d2J_dpix_dy = 2 * cos(b) * sin(a);
    d2J_dpix_dz = -2 * sin(b);
    d2J_dpix_da =
        cos(b) * sin(a) *
            (2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
             2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             2 * pix * cos(a) * cos(b)) -
        cos(a) * cos(b) *
            (2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * pix * cos(b) * sin(a)) -
        2 * cos(b) * sin(a) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) +
        2 * cos(a) * cos(b) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a));
    d2J_dpix_db =
        sin(b) * (2 * pix * cos(b) + 2 * piz * cos(c) * sin(b) +
                  2 * piy * sin(b) * sin(c)) -
        2 * cos(b) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c)) +
        cos(a) * cos(b) *
            (2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
             2 * piy * cos(a) * cos(b) * sin(c)) -
        2 * sin(a) * sin(b) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) +
        cos(b) * sin(a) *
            (2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
             2 * piy * cos(b) * sin(a) * sin(c)) -
        2 * cos(a) * sin(b) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b));
    d2J_dpix_dc =
        cos(a) * cos(b) *
            (2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) -
        sin(b) * (2 * piy * cos(b) * cos(c) - 2 * piz * cos(b) * sin(c)) -
        cos(b) * sin(a) *
            (2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)));
    d2J_dpiy_dx = 2 * cos(a) * sin(b) * sin(c) - 2 * cos(c) * sin(a);
    d2J_dpiy_dy = 2 * cos(a) * cos(c) + 2 * sin(a) * sin(b) * sin(c);
    d2J_dpiy_dz = 2 * cos(b) * sin(c);
    d2J_dpiy_da =
        (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) *
            (2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
             2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             2 * pix * cos(a) * cos(b)) +
        (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) *
            (2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * pix * cos(b) * sin(a)) -
        (2 * cos(a) * cos(c) + 2 * sin(a) * sin(b) * sin(c)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) -
        (2 * cos(c) * sin(a) - 2 * cos(a) * sin(b) * sin(c)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a));
    d2J_dpiy_db =
        (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) *
            (2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
             2 * piy * cos(b) * sin(a) * sin(c)) -
        (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) *
            (2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
             2 * piy * cos(a) * cos(b) * sin(c)) -
        2 * sin(b) * sin(c) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c)) -
        cos(b) * sin(c) *
            (2 * pix * cos(b) + 2 * piz * cos(c) * sin(b) +
             2 * piy * sin(b) * sin(c)) +
        2 * cos(a) * cos(b) * sin(c) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) +
        2 * cos(b) * sin(a) * sin(c) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a));
    d2J_dpiy_dc =
        (2 * sin(a) * sin(c) + 2 * cos(a) * cos(c) * sin(b)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) -
        (2 * cos(a) * sin(c) - 2 * cos(c) * sin(a) * sin(b)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) -
        (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) *
            (2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) -
        (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) *
            (2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) +
        2 * cos(b) * cos(c) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c)) +
        cos(b) * sin(c) *
            (2 * piy * cos(b) * cos(c) - 2 * piz * cos(b) * sin(c));
    d2J_dpiz_dx = 2 * sin(a) * sin(c) + 2 * cos(a) * cos(c) * sin(b);
    d2J_dpiz_dy = 2 * cos(c) * sin(a) * sin(b) - 2 * cos(a) * sin(c);
    d2J_dpiz_dz = 2 * cos(b) * cos(c);
    d2J_dpiz_da =
        (2 * cos(a) * sin(c) - 2 * cos(c) * sin(a) * sin(b)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) -
        (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) *
            (2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * pix * cos(b) * sin(a)) -
        (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) *
            (2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
             2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             2 * pix * cos(a) * cos(b)) +
        (2 * sin(a) * sin(c) + 2 * cos(a) * cos(c) * sin(b)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a));
    d2J_dpiz_db =
        (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) *
            (2 * piz * cos(a) * cos(b) * cos(c) - 2 * pix * cos(a) * sin(b) +
             2 * piy * cos(a) * cos(b) * sin(c)) -
        (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) *
            (2 * piz * cos(b) * cos(c) * sin(a) - 2 * pix * sin(a) * sin(b) +
             2 * piy * cos(b) * sin(a) * sin(c)) -
        2 * cos(c) * sin(b) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c)) -
        cos(b) * cos(c) *
            (2 * pix * cos(b) + 2 * piz * cos(c) * sin(b) +
             2 * piy * sin(b) * sin(c)) +
        2 * cos(a) * cos(b) * cos(c) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) +
        2 * cos(b) * cos(c) * sin(a) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a));
    d2J_dpiz_dc =
        (2 * cos(c) * sin(a) - 2 * cos(a) * sin(b) * sin(c)) *
            (x - qix - piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) +
             piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             pix * cos(a) * cos(b)) -
        (2 * cos(a) * cos(c) + 2 * sin(a) * sin(b) * sin(c)) *
            (y - qiy + piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
             piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             pix * cos(b) * sin(a)) +
        (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) *
            (2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) +
             2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c))) +
        (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) *
            (2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
             2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c))) +
        cos(b) * cos(c) *
            (2 * piy * cos(b) * cos(c) - 2 * piz * cos(b) * sin(c)) -
        2 * cos(b) * sin(c) *
            (z - qiz - pix * sin(b) + piz * cos(b) * cos(c) +
             piy * cos(b) * sin(c));
    d2J_dqix_dx = -2;
    d2J_dqix_dy = 0;
    d2J_dqix_dz = 0;
    d2J_dqix_da = 2 * piy * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c)) -
                  2 * piz * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
                  2 * pix * cos(b) * sin(a);
    d2J_dqix_db = 2 * pix * cos(a) * sin(b) -
                  2 * piz * cos(a) * cos(b) * cos(c) -
                  2 * piy * cos(a) * cos(b) * sin(c);
    d2J_dqix_dc = -2 * piy * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                  2 * piz * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c));
    d2J_dqiy_dx = 0;
    d2J_dqiy_dy = -2;
    d2J_dqiy_dz = 0;
    d2J_dqiy_da = 2 * piy * (cos(c) * sin(a) - cos(a) * sin(b) * sin(c)) -
                  2 * piz * (sin(a) * sin(c) + cos(a) * cos(c) * sin(b)) -
                  2 * pix * cos(a) * cos(b);
    d2J_dqiy_db = 2 * pix * sin(a) * sin(b) -
                  2 * piz * cos(b) * cos(c) * sin(a) -
                  2 * piy * cos(b) * sin(a) * sin(c);
    d2J_dqiy_dc = 2 * piy * (cos(a) * sin(c) - cos(c) * sin(a) * sin(b)) +
                  2 * piz * (cos(a) * cos(c) + sin(a) * sin(b) * sin(c));
    d2J_dqiz_dx = 0;
    d2J_dqiz_dy = 0;
    d2J_dqiz_dz = -2;
    d2J_dqiz_da = 0;
    d2J_dqiz_db = 2 * pix * cos(b) + 2 * piz * cos(c) * sin(b) +
                  2 * piy * sin(b) * sin(c);
    d2J_dqiz_dc = 2 * piz * cos(b) * sin(c) - 2 * piy * cos(b) * cos(c);
    d2J_dZdX_temp << d2J_dpix_dx, d2J_dpiy_dx, d2J_dpiz_dx, d2J_dqix_dx,
        d2J_dqiy_dx, d2J_dqiz_dx, d2J_dpix_dy, d2J_dpiy_dy, d2J_dpiz_dy,
        d2J_dqix_dy, d2J_dqiy_dy, d2J_dqiz_dy, d2J_dpix_dz, d2J_dpiy_dz,
        d2J_dpiz_dz, d2J_dqix_dz, d2J_dqiy_dz, d2J_dqiz_dz, d2J_dpix_da,
        d2J_dpiy_da, d2J_dpiz_da, d2J_dqix_da, d2J_dqiy_da, d2J_dqiz_da,
        d2J_dpix_db, d2J_dpiy_db, d2J_dpiz_db, d2J_dqix_db, d2J_dqiy_db,
        d2J_dqiz_db, d2J_dpix_dc, d2J_dpiy_dc, d2J_dpiz_dc, d2J_dqix_dc,
        d2J_dqiy_dc, d2J_dqiz_dc;
    d2J_dZdX.block<6, 6>(0, 6 * k) = d2J_dZdX_temp;
  }

  // By reaching here both the matrices d2J_dX2 and d2J_dZdX are calculated and
  // lets print those values out;

  // std::cout << "\n Finally here are the results \n\n" << "d2J_dX2 = \n " <<
  // d2J_dX2 <<std::endl; std::cout << "\n\n\n" << "d2J_dZdX = \n " << d2J_dZdX
  // <<std::endl;

  // By reaching here both the matrices d2J_dX2 and d2J_dZdX are calculated and
  // lets print those values out;

  // std::cout << "\n Finally here are the two matrices \n\n" << "d2J_dX2 = \n "
  // << d2J_dX2 <<std::endl; std::cout << "\n\n\n" << "d2J_dZdX = \n " <<
  // d2J_dZdX <<std::endl;

  /**************************************
   *
   * Here we create the matrix cov(z) as mentioned in Section 3.3 in the paper,
   * "Covariance of ICP with 3D Point to Point and Point to Plane Error Metrics"
   *
   * ************************************/

  Eigen::MatrixXd cov_z(6 * n, 6 * n);
  cov_z = Eigen::MatrixXd::Identity(6 * n, 6 * n);
  for (size_t i = 0; i < p_cov.size(); ++i) {
    cov_z.block<3, 3>(i * 6, i * 6) = p_cov[i];
    cov_z.block<3, 3>(i * 6 + 3, i * 6 + 3) = q_cov[i];
  }

  // std::cout << "\n\n********************** \n\n" << "cov_z = \n" << cov_z
  // <<"\n*******************\n\n"<< std::endl;

  *ICP_COV = d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() *
             d2J_dX2.inverse();

  // std::cout << "\nSuccessfully Computed the ICP's Covariance !!!\n" <<
  // std::endl;
}

}  // namespace semantify
#endif  // SEMANTIFY_PLUGIN_ICP_COVARIANCE_H_
