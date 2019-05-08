#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#include "Eigen/Dense"

class SensorData {
public:
  long long timestamp_;
  Eigen::VectorXd data_;
};

#endif /* SENSOR_DATA_H_ */
