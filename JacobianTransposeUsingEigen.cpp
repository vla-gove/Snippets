// inverse kinematics with jacobian transpose using eigen library
std::vector<double> inverseKinematics(Eigen::Vector3d target) {
  int maxIterations = 100;
  double errorTolerance = 1e-6;
  
  std::vector<double> jointAngles(joints.size(), 0);
  Eigen::Matrix4d T = forwardKinematics(jointAngles);
  Eigen::Vector3d error = target - T.block<3,1>(0,3);
  
  for (int i = 0; i < maxIterations && error.norm() > errorTolerance; i++) {
    Eigen::MatrixXd J(3, joints.size());
    for (int j = 0; j < joints.size(); j++) {
      Eigen::Vector3d z_prev, p_prev;
      if (j == 0) {
        z_prev << 0, 0, 1;
        p_prev = Eigen::Vector3d::Zero();
      } else {
        z_prev = T.block<3,1>(0,2);
        p_prev = T.block<3,1>(0,3);
      }
      
      Eigen::Matrix4d T_prev = T;
      jointAngles[j] += 0.1;
      T = forwardKinematics(jointAngles);
      Eigen::Vector3d z_next = T.block<3,1>(0,2);
      Eigen::Vector3d p_next = T.block<3,1>(0,3);
      J.col(j) = z_prev.cross(p_next - p_prev);
      
      jointAngles[j] -= 0.1;
      T = T_prev;
    }
    
    Eigen::VectorXd dTheta = J.transpose() * error;
    for (int j = 0; j < joints.size(); j++) {
      jointAngles[j] += dTheta[j];
    }
    T = forwardKinematics(jointAngles);
    error = target - T.block<3,1>(0,3);
  }
  
  return jointAngles;
}