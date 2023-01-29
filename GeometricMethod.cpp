// calculating inverse kinematics of a 6dof robot using geometric method
std::array<double, 6> InverseKinematics::calculate(std::array<double, 3> ee_pos) {
    std::array<double, 6> joint_angles;

    // position of the end effector in the base frame
    std::array<double, 3> ee_pos_base = { ee_pos[0], ee_pos[1], ee_pos[2] };

    // position of the wrist center
    std::array<double, 3> wrist_center_pos = { ee_pos_base[0] - robot_.get_dh_param(5).d * ee_pos_base[2],
                                               ee_pos_base[1] - robot_.get_dh_param(5).d * ee_pos_base[2],
                                               0 };

    // first three joint angles using geometric method
    joint_angles[0] = atan2(wrist_center_pos[1], wrist_center_pos[0]);
    double d = sqrt(pow(wrist_center_pos[0], 2) + pow(wrist_center_pos[1], 2));
    double a1 = robot_.get_dh_param(0).a;
    double a2 = robot_.get_dh_param(1).a;
    double a3 = robot_.get_dh_param(2).a;
    double c2 = (pow(d, 2) - pow(a1, 2) - pow(a2, 2)) / (2 * a1 * a2);
    double s2 = sqrt(1 - pow(c2, 2));
    joint_angles[1] = atan2(s2, c2);
    double k1 = a1 + a2 * c2;
    double k2 = a2 * s2;
    joint_angles[2] = atan2(wrist_center_pos[2], d) - atan2(k2, k1);

    // last three joint angles using geometric method
    std::array<std::array<double, 3>, 3> frames;
    frames[0] = { cos(joint_angles[0]), -sin(joint_angles[0]), 0,
                  sin(joint_angles[0]), cos(joint_angles[0]), 0,
                  0, 0, 1 };
    double c1 = cos(joint_angles[1]);
    double s1 = sin(joint_angles[1]);
    frames[1] = { c1, -s1, 0,
                  s1 * cos(robot_.get_dh_param(0).alpha), c1 * cos(robot_.get_dh_param(0).alpha), -sin(robot_.get_dh_param(0).alpha),
                  s1 * sin(robot_.get_dh_param(0).alpha), c1 * sin(robot_.get_dh_param(0).alpha), cos(robot_.get_dh_param(0).alpha) };
    double c2 = cos(joint_angles[2]);
    double s2 = sin(joint_angles[2]);
    frames[2] = { c2, -s2, 0,
                  s2 * cos(robot_.get_dh_param(1).alpha), c2 * cos(robot_.get_dh_param(1).alpha), -sin(robot_.get_dh_param(1).alpha),
                  s2 * sin(robot_.get_dh_param(1).alpha), c2 * sin(robot_.get_dh_param(1).alpha), cos(robot_.get_dh_param(1).alpha) };

    // rotation matrix of the end effector in the base frame
    std::array<std::array<double, 3>, 3> R_ee_base;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_ee_base[i][j] = frames[0][i*3+j]*frames[1][i*3+j]*frames[2][i*3+j];
        }
    }

    // last three joint angles using geometric method
    double c3 = R_ee_base[0][2];
    double s3 = -R_ee_base[1][2];
    joint_angles[3] = atan2(s3, c3);
    double c4 = sqrt(pow(R_ee_base[0][2], 2) + pow(R_ee_base[1][2], 2));
    double s4 = R_ee_base[2][2];
    joint_angles[4] = atan2(s4, c4);
    double c5 = R_ee_base[0][0];
    double s5 = R_ee_base[1][0];
    joint_angles[5] = atan2(s5, c5);

    return joint_angles;
}