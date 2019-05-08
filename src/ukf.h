#ifndef UKF_H
#define UKF_H

#include "sensor_data.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Number of sigma points
  int n_aug_sigma_;

  // Sigma point spreading parameter
  double lambda_;

  // the current NIS for laser
  double NIS_laser_;

  // Weights of sigma points
  VectorXd weights_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // predicted sigma points matrix
  MatrixXd Xsig_pred_;

  // augmented sigma points matrix
  MatrixXd Xsig_aug_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(SensorData meas_package);

  /**
   * Initialize the state, covariance matrix and timestamp using a radar measurement
   * @param meas_package The first measurement
   */
  void Initialization(SensorData meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(SensorData meas_package);

  /**
   * Predict either lidar or radar measurement with given Sigma predictions
   * @param n_z The measurement dimension
   * @param Zsig The matrix for sigma points in measurement space
   * @param z_pred The predicted measurement mean
   * @param S The measurement covariance matrix
   * @param R The measurement noise covariance matrix
   */
  void PredictMeasurement(int n_z, const MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, MatrixXd &R) ;
  /**
   * Updates the state with either lidar or radar measurement
   * @param z The measurement at k+1
   * @param z_pred The predictionof measurement at k+1
   * @param S The measurement covariance matrix
   * @param Zsig The matrix for sigma points in measurement space
   */
  void UpdateState(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S, const MatrixXd &Zsig);

  /**
   * Calculate augmented sigma points: Xsig_agu_
   */
  void AugmentSigmaPoints();


  /**
   * Predict the sigma points: Xsig_pred_
   * @param delta_t Time between k and k+1 in s
   */
  void PredictSigmaPoints(double delta_t);

  /**
   * Predict Mean and Covariance of the predicted state: x_ and P_
   */
  void PredictMeanAndCovariance();
};

#endif /* UKF_H */
