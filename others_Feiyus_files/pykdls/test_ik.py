
import rospy

from baxter_pykdl import baxter_kinematics

def main():
    rospy.init_node('baxter_kinematics')

    print '*** Baxter PyKDL Kinematics ***\n'
    kin = baxter_kinematics('right')

    print '\n*** Baxter Description ***\n'
    kin.print_robot_description()
    print '\n*** Baxter KDL Chain ***\n'
    kin.print_kdl_chain()
    # FK Position
    print '\n*** Baxter Position FK ***\n'
    print kin.forward_position_kinematics()
    # FK Velocity
    # print '\n*** Baxter Velocity FK ***\n'
    # kin.forward_velocity_kinematics()
    # IK
    print '\n*** Baxter Position IK ***\n'
    pos = [0.582583, -0.180819, 0.216003]
    rot = [0.03085, 0.9945, 0.0561, 0.0829]
    print kin.inverse_kinematics(pos)  # position, don't care orientation
    print '\n*** Baxter Pose IK ***\n'
    print kin.inverse_kinematics(pos, rot)  # position & orientation
    # Jacobian
    print '\n*** Baxter Jacobian ***\n'
    print kin.jacobian()
    # Jacobian Transpose
    print '\n*** Baxter Jacobian Tranpose***\n'
    print kin.jacobian_transpose()
    # Jacobian Pseudo-Inverse (Moore-Penrose)
    print '\n*** Baxter Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
    print kin.jacobian_pseudo_inverse()
    # Joint space mass matrix
    print '\n*** Baxter Joint Inertia ***\n'
    print kin.inertia()
    # Cartesian space mass matrix
    print '\n*** Baxter Cartesian Inertia ***\n'
    print kin.cart_inertia()

if __name__ == "__main__":
    main()
