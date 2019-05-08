#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "sensor_data.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]);

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name);

void readFileData(ifstream &in_file_, vector<SensorData> &measurement_pack_list, 
                  vector<SensorData> &gt_pack_list);

void addOutFileHead(ofstream& out_file_);

Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                              const std::vector<Eigen::VectorXd> &ground_truth);

void outputData(ofstream& out_file_, UKF &ukf,
                SensorData &measurement, SensorData &ground_truth);
                
void recordEstimation(UKF &ukf, SensorData &gt, vector<VectorXd> &estimations, 
                      vector<VectorXd> &ground_truth);

int main(int argc, char* argv[]) 
{
  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/
  vector<SensorData> measurement_pack_list;
  vector<SensorData> gt_pack_list;
  readFileData(in_file_, measurement_pack_list, gt_pack_list);

  // Create a UKF instance
  UKF ukf;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  addOutFileHead(out_file_);
  for (size_t k = 0; k < measurement_pack_list.size(); ++k) 
  {
    // Call the UKF-based fusion
    ukf.ProcessMeasurement(measurement_pack_list[k]);

    outputData(out_file_, ukf, measurement_pack_list[k], gt_pack_list[k]);

    recordEstimation(ukf, gt_pack_list[k], estimations, ground_truth);
  }

  // compute the accuracy (RMSE)
  std::cout << "RMSE" << endl << CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) out_file_.close();
  if (in_file_.is_open()) in_file_.close();

  std::cout << "Done!" << endl;
  return 0;
}

void recordEstimation(UKF &ukf, SensorData &gt, vector<VectorXd> &estimations, vector<VectorXd> &ground_truth)
{
  float x_estimate_ = ukf.x_(0);
  float y_estimate_ = ukf.x_(1);
  float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
  float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));
  
  VectorXd ukf_x_cartesian_ = VectorXd(4);
  ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;
  
  estimations.push_back(ukf_x_cartesian_);
  ground_truth.push_back(gt.data_);
}

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

void readFileData(ifstream &in_file_, vector<SensorData> &measurement_pack_list, vector<SensorData> &gt_pack_list)
{
  string line;
  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    SensorData meas_package;
    SensorData gt_package;
    istringstream iss(line);
    long long timestamp;

    // read measurements at this timestamp
    meas_package.data_ = VectorXd(2);
    float px;
    float py;
    iss >> px;
    iss >> py;
    meas_package.data_ << px, py;

    iss >> timestamp;
    meas_package.timestamp_ = timestamp;

    measurement_pack_list.push_back(meas_package);
   
    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.data_ = VectorXd(4);
    gt_package.data_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }
}

void addOutFileHead(ofstream& out_file_)
{
// column names for output file
  out_file_ << "time_stamp" << "\t";  
  out_file_ << "px_state" << "\t";
  out_file_ << "py_state" << "\t";
  out_file_ << "v_state" << "\t";
  out_file_ << "yaw_angle_state" << "\t";
  out_file_ << "yaw_rate_state" << "\t";
  out_file_ << "NIS" << "\t";  
  out_file_ << "px_measured" << "\t";
  out_file_ << "py_measured" << "\t";
  out_file_ << "px_ground_truth" << "\t";
  out_file_ << "py_ground_truth" << "\t";
  out_file_ << "vx_ground_truth" << "\t";
  out_file_ << "vy_ground_truth" << "\n";
}

Eigen::VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}


void outputData(ofstream& out_file_, UKF &ukf, SensorData &measurement, SensorData &ground_truth)
{
  // timestamp
  out_file_ << measurement.timestamp_ << "\t"; // pos1 - est

  // output the state vector
  out_file_ << ukf.x_(0) << "\t"; // pos1 - est
  out_file_ << ukf.x_(1) << "\t"; // pos2 - est
  out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
  out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
  out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

  // NIS value
  out_file_ << ukf.NIS_laser_ << "\t";

  // output the lidar sensor measurement px and py
  out_file_ << measurement.data_(0) << "\t";
  out_file_ << measurement.data_(1) << "\t";

  // output the ground truth
  out_file_ << ground_truth.data_(0) << "\t";
  out_file_ << ground_truth.data_(1) << "\t";
  out_file_ << ground_truth.data_(2) << "\t";
  out_file_ << ground_truth.data_(3) << "\n";
}
