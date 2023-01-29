// jacobian inverse using eigen library
std::vector<double> Robot::inverse_kinematics(std::vector<double> end_effector_position)
{
    std::vector<double> initial_guess(dof); // initial joint angles
    std::vector<double> joint_angles = initial_guess;
    Eigen::MatrixXd jacobian_matrix(6,dof);
    double error=1;
    double tolerance=0.0001;
    int max_iterations=100;
    int iteration=0;
    while(error>tolerance && iteration<max_iterations)
    {
        // forward kinematics
        std::vector<double> fk = forward_kinematics(joint_angles);
        // error
        std::vector<double> ee_error(6);
        for(int i=0;i<3;i++)
            ee_error[i]=end_effector_position[i]-fk[i];
        for(int i=3;i<6;i++)
            ee_error[i]=0-fk[i];
        error = ee_error.norm();
        // jacobian
        jacobian_matrix=jacobian(joint_angles);
        //calculate delta_theta
        Eigen::VectorXd delta_theta(dof);
        Eigen::VectorXd ee_error_eigen(6);
        for(int i=0;i<6;i++)
            ee_error_eigen(i)=ee_error[i];
        delta_theta=jacobian_matrix.colPivHouseholderQr().solve(ee_error_eigen);
        // theta
        for(int i=0;i<dof;i++)
            joint_angles[i]+=delta_theta(i);
        iteration++;
    }
    return joint_angles;
}
