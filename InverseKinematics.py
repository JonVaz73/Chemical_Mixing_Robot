import numpy as np
from ikpy.chain import Chain
from ikpy.link import DHLink

class XArmIK:
    def __init__(self):
        # Define the kinematic chain using DH parameters
        self.chain = Chain(name='xArm 1S', links=[
            DHLink(d=30, a=0, alpha=np.pi/2),
            DHLink(d=0, a=101, alpha=0),
            DHLink(d=0, a=95, alpha=0),
            DHLink(d=0, a=0, alpha=np.pi/2),
            DHLink(d=53, a=0, alpha=0),
            DHLink(d=105, a=0, alpha=0)
        ])

    def calculate_joint_angles(self, target_position):
        """
        Computes the joint angles for a given target position using inverse kinematics.
        :param target_position: A list of x, y, z coordinates [x, y, z].
        :return: A list of joint angles in radians.
        """
        try:
            joint_angles = self.chain.inverse_kinematics(target_position)
            return joint_angles
        except Exception as e:
            print(f"Error computing IK: {e}")
            return None

    def calculate_forward_kinematics(self, joint_angles):
        """
        Computes the end-effector position for a given set of joint angles using forward kinematics.
        :param joint_angles: A list of joint angles in radians, reversed (gripper first, base last).
        :return: A list [x, y, z] representing the position of the end effector.
        """
        try:
            # Reverse the joint angles to match the DH parameter order
            joint_angles_correct_order = joint_angles[::-1]  # Reverse the list

            # Compute forward kinematics
            fk_result = self.chain.forward_kinematics(joint_angles_correct_order)

            # Extract the position from the transformation matrix
            x, y, z = fk_result[:3, 3]
            return [x, y, z]
        except Exception as e:
            print(f"Error computing FK: {e}")
            return None
