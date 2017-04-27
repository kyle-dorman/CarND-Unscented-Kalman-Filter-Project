#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  int lambda_;

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
  void ProcessMeasurement(MeasurementPackage meas_package);

private:
  /**
   * Initializes state with first measurement.
   * @param meas_package the measurement.
   */
  void Initialize(MeasurementPackage meas_package);

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
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Calculated the updated sigma points. 
   */
  MatrixXd AugmentedSigmaPoints();

  /**
   * Predicts the sigma points change based on time since last measurement.
   * @param delta_t The time change in seconds.
   */
  void PredictSigmaPoints(double delta_t);

  /**
   * Based on the saved predicted sigma points, calculate the new mean and covariance.
   */
  void PredictMeanAndCovariance();

  /**
   * Generic function to update the sate based on the measurement.
   * @param n_z Degrees of freedom.
   * @param Zsig sigma points in measurement space.
   * @param R measurement noise covariance matrix.
   * @param meas_package The measurement at k+1.
   * @param NIS pointer to NIS value to update (laser or radar).
   */
  void UpdateState(int n_z, MatrixXd Zsig, MatrixXd R, MeasurementPackage meas_package, double* NIS);

  /**
   * Calculate the difference between the actual and predicted values.
   *  If it is a radar measurement, normalize the angle.
   * @param x measured x val.
   * @param x_pred predicted x val.
   * @param sensorType Laser or Radar.
   */
  VectorXd XDiff(VectorXd x, VectorXd x_pred, MeasurementPackage::SensorType sensorType);

  /**
   * Normalizes nangle between (-)pi and pi.
   * @param angle angle in radians.
   */
  double NormalizeAngle(double angle);
};

#endif /* UKF_H */
