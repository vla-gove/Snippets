// calculating inverse kinematics of a 6dof robot using inverse jacobian method
std::array<double, 6> RobotManipulator::calculate(std::array<double, 3> end_effector_pos) {
    std::array<double, 6> joint_angles;
    double error = 0.01;
    int max_iter = 100;
    std::array<double, 3> ee_pos;
    std::array<double, 3> ee_pos_prev;
    std::array<std::array<double, 3>, 6> jacobian;
    std::array<std::array<double, 3>, 6> jacobian_inv;
    std::array<std::array<double, 3>, 6> jacobian_transpose;
    std::array<std::array<double, 3>, 6> jacobian_inv_transpose;
    std::array<double, 3> ee_pos_error;
    std::array<double, 3> joint_angles_change;

    for (int i = 0; i < max_iter; i++) {
        // jacobian matrix
        for (int j = 0; j < 6; j++) {
            std::array<double, 3> joint_axis;
            std::array<double, 3> link_pos;
            double c, s;
            c = cos(joint_angles[j]);
            s = sin(joint_angles[j]);

            // link position
            link_pos[0] = robot_.get_dh_param(j).a * c;
            link_pos[1] = robot_.get_dh_param(j).a * s;
            link_pos[2] = 0;

            // calculate joint axis
            joint_axis[0] = -s * robot_.get_dh_param(j).alpha;
            joint_axis[1] = c * robot_.get_dh_param(j).alpha;
            joint_axis[2] = 0;

            for (int k = 0; k < 3; k++) {
                jacobian[j][k] = joint_axis[k];
                if (k < 2) {
                    jacobian[j][k] *= robot_.get_dh_param(j).d;
                } else {
                    jacobian[j][k] *= robot_.get_dh_param(j).a;
                }
            }
        }

        // inverse of jacobian matrix
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 6; k++) {
                jacobian_transpose[j][k] = jacobian[k][j];
            }
        }
        double det = jacobian[0][0] * (jacobian[1][1] * jacobian[2][2] - jacobian[2][1] * jacobian[1][2]) -
        jacobian[0][1] * (jacobian[jacobian[1][0] * jacobian[2][2] - jacobian[2][0] * jacobian[1][2]) +
                                                                                                              jacobian[0][2] * (jacobian[1][0] * jacobian[2][1] - jacobian[2][0] * jacobian[1][1]);


        jacobian_inv[0][0] = (jacobian[1][1] * jacobian[2][2] - jacobian[2][1] * jacobian[1][2]) / det;
        jacobian_inv[0][1] = (jacobian[0][2] * jacobian[2][1] - jacobian[0][1] * jacobian[2][2]) / det;
        jacobian_inv[0][2] = (jacobian[0][1] * jacobian[1][2] - jacobian[0][2] * jacobian[1][1]) / det;
        jacobian_inv[1][0] = (jacobian[1][2] * jacobian[2][0] - jacobian[1][0] * jacobian[2][2]) / det;
        jacobian_inv[1][1] = (jacobian[0][0] * jacobian[2][2] - jacobian[0][2] * jacobian[2][0]) / det;
        jacobian_inv[1][2] = (jacobian[1][0] * jacobian[0][2] - jacobian[0][0] * jacobian[1][2]) / det;
        jacobian_inv[2][0] = (jacobian[1][0] * jacobian[2][1] - jacobian[2][0] * jacobian[1][1]) / det;
        jacobian_inv[2][1] = (jacobian[2][0] * jacobian[0][1] - jacobian[0][0] * jacobian[2][1]) / det;
        jacobian_inv[2][2] = (jacobian[0][0] * jacobian[1][1] - jacobian[1][0] * jacobian[0][1]) / det;

        // calculate the inverse of jacobian matrix transpose
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 3; k++) {
                jacobian_inv_transpose[j][k] = jacobian_inv[k][j];
            }
        }

        // calculate end effector position
        ee_pos = forward_kinematics(joint_angles);

        // calculate end effector position error
        for (int j = 0; j < 3; j++) {
            ee_pos_error[j] = end_effector_pos[j] - ee_pos[j];
        }

        // calculate change in joint angles

        for (int j = 0; j < 6; j++) {
            delta_theta[j] = 0;
            for (int k = 0; k < 3; k++) {
                delta_theta[j] += jacobian_inv_transpose[j][k] * ee_pos_error[k];
            }
        }


        // update joint angles
        for (int j = 0; j < 6; j++) {
            joint_angles[j] += delta_theta[j];
        }

        // check for convergence
        if (norm(ee_pos_error) < error_tolerance) {
            converged = true;
        }
    }

    return joint_angles;
}

