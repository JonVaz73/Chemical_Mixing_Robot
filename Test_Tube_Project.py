from InverseKinematics import XArmIK
import xarm
import time


# Initialize the XArmIK class
Hiwonder = XArmIK()

# Initialize the robot instance
robot = xarm.Controller('USB')

#Max and Min Angles of our Arm
max_angles = 125
min_angles = -125

def execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay,adjust_angle):
    error = False
    error_count = 0
    joint_angles = joint_angles_degrees
    motor_delay = time_delay

    if adjust_angle == True:
        for action in joint_angles:

            # setting the joint angles
            link_Base = action[5]
            link_1 = 90.0 - action[2]
            link_2 = action[3]
            link_3 = -(action[4])
            link_hand = action[1]
            link_gripper = action[0]

            # check angles
            for i in range(len(action)):
                if int(action[i]) > 125 or int(action[i]) < -125:
                    print(f"Error: Joint angle {i + (6 - i)} with value of {action[i]}")
                    error = True
                    error_count += 1
            if error == True:
                print(f"Uh Ohh .... looks like some errors occured, {error_count} number of errors to be exact...")
                print("Point you are trying to reach is NOT in work Space ... ")
                print("Fix the damn errors! but have a good day")
                program_running = False
            elif error == False:
                print("All Angles look good...Proceeding to Point")
                [x, y, z] = Hiwonder.calculate_forward_kinematics(action)
                print(f"The x,y,z position is as follows: {x},{y},{z}")
                robot.setPosition(1, link_gripper, wait=False)
                robot.setPosition(2, link_hand, wait=False)
                robot.setPosition(3, link_3, wait=False)
                robot.setPosition(4, link_2, wait=False)
                robot.setPosition(5, link_1, wait=False)
                robot.setPosition(6, link_Base, wait=False)
            time.sleep(motor_delay)
    elif adjust_angle == False:
        for action in joint_angles_degrees:

            # setting the joint angles
            link_Base = action[5]
            link_2 = action[4]
            link_3 = action[3]
            link_4 = action[2]
            link_hand = action[1]
            link_gripper = action[0]

            # check angles
            for i in range(len(action)):
                if int(action[i]) > 125 or int(action[i]) < -125:
                    print(f"Error: Joint angle {i + (6 - i)} with value of {action[i]}")
                    error = True
                    error_count += 1
            if error == True:
                print(f"Uh Ohh .... looks like some errors occured, {error_count} number of errors to be exact...")
                print("Point you are trying to reach is NOT in work Space ... ")
                print("Fix the damn errors! but have a good day")
                program_running = False
            elif error == False:
                print("All Angles look good...Proceeding to Point")
                [x, y, z] = Hiwonder.calculate_forward_kinematics(action)
                print(f"The x,y,z position is as follows: {x},{y},{z}")
                robot.setPosition(1, link_gripper, wait=False)
                robot.setPosition(2, link_hand, wait=False)
                robot.setPosition(3, link_4, wait=False)
                robot.setPosition(4, link_3, wait=False)
                robot.setPosition(5, link_2, wait=False)
                robot.setPosition(6, link_Base, wait=False)
            time.sleep(motor_delay)

print("Setting initial positions to home rest...")
home_position_open = [0.0, -5.0, -42.0, -125.0, 0.0, -3.0]               #[0.0, 0.0, 46.0,-70.0,-29.0,0.0]
joint_angles_degrees = [home_position_open]
time_delay = 2
adjust_angles = False
execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay,adjust_angles)
print("Initial positions set.")
time.sleep(2)

#Define Key positions
home_position_closed = [125.0, 0.0, 46.0, -70.0, -29.0, 0.0]
Pre_pour_position = [125.0, 0.0, -5.0, -104.0, -11.0, -29.0]
Pour_position = [125.0, 125.0, -5.0, -104.0, -11.0, -39.0]
#Tube2
Pre_pickup_tube2 = [0.0, -5.0, -42.0, -125.0, 0.0, -3.0]
Pickup2_open = [ 0.0 , -5.0 , -36.0, -96.0, 28.0, -3.0 ]
Pickup2_closed = [ 125.0 , -5.0 , -36.0, -96.0, 28.0, -3.0 ]
Tube2_lift1 = [ 125.0 , -5.0 , -33.0, -91.0, 27.0, -3.0 ]
Tube2_lift2 = [ 125.0 , -5.0 , -25.0, -90.0, 22.0, -3.0 ]
Tube2_lift3 = [ 125.0 , -5.0 , -23.0, -89.0, 17.0, -3.0 ]
#tube1
Pre_pickup_tube1 = [0.0, -5.0, -40.0, -125.0, 0.0, 7.0]
Pickup1_open = [ 0.0 , -5.0 , -40.0, -96.0, 32.0, 7.0 ]
Pickup1_closed = [ 125.0 , -5.0 , -40.0, -96.0, 32.0, 7.0 ]
Tube1_lift1 = [ 125.0 , -5.0 , -33.0, -91.0, 27.0, 7.0 ]
Tube1_lift2 = [ 125.0 , -9.0 , -25.0, -90.0, 22.0, 7.0 ]
Tube1_lift3 = [ 125.0 , -9.0 , -19.0, -89.0, 17.0, 7.0 ]
#tube3
Pre_pickup_tube3 = [0.0, -5.0, -40.0, -125.0, 0.0, -10.0]
Pickup3_open = [ 0.0 , -5.0 , -40.0, -96.0, 32.0, -10.0 ]
Pickup3_closed = [ 125.0 , -5.0 , -40.0, -96.0, 32.0, -10.0 ]
Tube3_lift1 = [ 125.0 , -5.0 , -33.0, -91.0, 27.0, -10.0 ]
Tube3_lift2 = [ 125.0 , -9.0 , -25.0, -90.0, 22.0, -10.0 ]
Tube3_lift3 = [ 125.0 , -9.0 , -19.0, -89.0, 17.0, -10.0 ]
#Creating offset for base
offset = -3.0

#InverseKinematics test points Tube 2 Lift
#[ gripper strength, grip rotation, link 1 angle, link 2, link 3, base rotation only for values from inverse kinematics
Test_start_open =  [ 0.0, -9.0 ,64.6359 , -95.5779  , 30.9420, -3.0 ]
Test_start_closed =  [ 125.0, -9.0 ,64.6359 , -95.5779  , 30.9420, -3.0 ]
Test_point1 =  [125.0, -9.0,  66.3542,  -93.9654,   27.6112, -3.0 ]
Test_point2 = [125.0, -9.0,   67.8874,  -92.1330,   24.2456, -3.0 ]
Test_point3 = [125.0, -9.0,   69.2303,  -90.0799,   20.8496, -3.0 ]
Test_point4 = [125.0, -9.0,   70.3784,  -87.8039,   17.4255, -3.0 ]
Test_point5 = [125.0, -9.0,   71.3276,  -85.3007,   13.9732, -3.0 ]
Test_point6 = [125.0, -9.0,   72.0736,  -82.5638,   10.4903, -3.0 ]
Test_point7 = [125.0, -9.0,   72.6116,  -79.5832,    6.9716, -3.0 ]
Test_point8 = [125.0, -9.0,   72.9355,  -76.3445,    3.4091, -3.0 ]
Test_point9 = [125.0, -9.0,   73.0369,  -72.8275,   -0.2093, -3.0 ]
#Tube3
Test3_start_open =  [ 0.0, -9.0 , 64.0453,  -94.6773,   30.6319, -10.0 ]
Test3_start_closed =  [ 125.0, -9.0 , 64.0453,  -94.6773,   30.6319, -10.0 ]
Test3_point1 =  [125.0, -9.0, 65.7455,  -93.0667,   27.3212, -10.0 ]
Test3_point2 = [125.0, -9.0,  67.2621,  -91.2356,   23.9734, -10.0 ]
Test3_point3 = [125.0, -9.0,  68.5899,  -89.1828,   20.5930, -10.0 ]
Test3_point4 = [125.0, -9.0,  69.7240,  -86.9059,   17.1819, -10.0 ]
Test3_point5 = [125.0, -9.0,  70.6601,  -84.4001,   13.7400, -10.0 ]
Test3_point6 = [125.0, -9.0,  71.3936,  -81.6582,   10.2646, -10.0 ]
Test3_point7 = [125.0, -9.0,  71.9194,  -78.6698,    6.7504, -10.0 ]
Test3_point8 = [125.0, -9.0,  72.2306,  -75.4196,    3.1889, -10.0 ]
Test3_point9 = [125.0, -9.0,  72.3184,  -71.8862,   -0.4322, -10.0 ]
#Tube1
Test1_start_open =  [ 0.0, -9.0 , 64.0453,  -94.6773,   30.6319, 7.0 ]
Test1_start_closed =  [ 125.0, -9.0 , 64.0453,  -94.6773,   30.6319, 7.0 ]
Test1_point1 =  [125.0, -9.0, 65.7455,  -93.0667,   27.3212, 7.0 ]
Test1_point2 = [125.0, -9.0,  67.2621,  -91.2356,   23.9734, 7.0 ]
Test1_point3 = [125.0, -9.0,  68.5899,  -89.1828,   20.5930, 7.0 ]
Test1_point4 = [125.0, -9.0,  69.7240,  -86.9059,   17.1819, 7.0 ]
Test1_point5 = [125.0, -9.0,  70.6601,  -84.4001,   13.7400, 7.0 ]
Test1_point6 = [125.0, -9.0,  71.3936,  -81.6582,   10.2646, 7.0 ]
Test1_point7 = [125.0, -9.0,  71.9194,  -78.6698,    6.7504, 7.0 ]
Test1_point8 = [125.0, -9.0,  72.2306,  -75.4196,    3.1889, 7.0 ]
Test1_point9 = [125.0, -9.0,  72.3184,  -71.8862,   -0.4322, 7.0 ]

program_running = True
while program_running:
    group_number = int(input('Enter Number 1 thru 3 to pick corresponding tube,enter 4 to verify pour position, 5 to enter sequence of tubes and any number outside to quit: '))
    if group_number == 4:
        print("Executing action group 4...")
        # Define the desired target position
        joint_angles_degrees = [home_position_closed, Pre_pour_position, Pour_position, Pre_pour_position,
                                home_position_closed]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)
        joint_angles_degrees = [Pre_pour_position, home_position_closed]
    elif group_number == 2:
        print("Executing action group 1...")
        # [ gripper strength, grip rotation, link 1 angle, link 2, link 3, base rotation]
        joint_angles_degrees = [Pre_pickup_tube2]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        joint_angles_degrees = [Test_start_open,Test_start_closed,Test_point1,Test_point2,Test_point3,Test_point4, Test_point5,
                                Test_point6, Test_point7, Test_point8, Test_point9]
        time_delay = 1
        adjust_angles = True
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        joint_angles_degrees = [home_position_closed, Pre_pour_position, Pour_position, Pre_pour_position,home_position_closed]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        joint_angles_degrees = [Test_point9, Test_point8, Test_point7, Test_point6, Test_point5, Test_point4,
                                Test_point3, Test_point2, Test_point1, Test_start_closed, Test_start_open]
        time_delay = 1
        adjust_angles = True
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        joint_angles_degrees = [Pre_pickup_tube2]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

    elif group_number == 1:
        print("Executing action group 2...")
        # Define the desired target position
        joint_angles_degrees = [Pre_pickup_tube1]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        joint_angles_degrees = [Test1_start_open, Test1_start_closed, Test1_point1, Test1_point2, Test1_point3, Test1_point4,Test1_point5,
                                Test1_point6, Test1_point7, Test1_point8, Test1_point9]
        time_delay = 1
        adjust_angles = True
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        joint_angles_degrees = [home_position_closed, Pre_pour_position, Pour_position, Pre_pour_position,
                                home_position_closed]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        joint_angles_degrees = [Test1_point9, Test1_point8, Test1_point7, Test1_point6, Test1_point5, Test1_point4,
                                Test1_point3, Test1_point2, Test1_point1, Test1_start_closed, Test1_start_open]
        time_delay = 1
        adjust_angles = True
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        joint_angles_degrees = [Pre_pickup_tube1]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)
    elif group_number == 3:
        print("Executing action group 3...")
        # Define the desired target position=
        joint_angles_degrees = [Pre_pickup_tube3]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)
        #set nect position to lift tube
        joint_angles_degrees = [Test3_start_open, Test3_start_closed, Test3_point1, Test3_point2, Test3_point3, Test3_point4,
                                Test3_point5, Test3_point6, Test3_point7, Test3_point8, Test3_point9]
        time_delay = 1
        adjust_angles = True
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles )
        #setting position to pour tube
        joint_angles_degrees = [home_position_closed, Pre_pour_position, Pour_position, Pre_pour_position,
                                home_position_closed]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay,adjust_angles)
        #setting position to replace tube
        joint_angles_degrees = [Test3_point9, Test3_point8, Test3_point7, Test3_point6, Test3_point5, Test3_point4,
                                Test3_point3, Test3_point2, Test3_point1, Test3_start_closed, Test3_start_open]
        time_delay = 1
        adjust_angles = True
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)
        #going back to home
        joint_angles_degrees = [Pre_pickup_tube2]
        time_delay = 2
        adjust_angles = False
        execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

    elif group_number == 5:
        print("Executing custom sequence...")
        sequence_input = input("Enter the sequence of action groups (e.g., 1,2,3): ")
        try:
            # Parse the sequence into a list of integers
            sequence = [int(num.strip()) for num in sequence_input.split(",")]

            # Validate the sequence contains valid group numbers
            if not all(1 <= num <= 3 for num in sequence):
                raise ValueError("Invalid group numbers. Please enter numbers between 1 and 3.")

            # Execute each group in the specified sequence
            for group in sequence:
                print(f"Executing action group {group}...")
                if group == 1:
                    # Execute action group 1
                    joint_angles_degrees = [Pre_pickup_tube1]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                    joint_angles_degrees = [Test1_start_open, Test1_start_closed, Test1_point1, Test1_point2,
                                            Test1_point3, Test1_point4, Test1_point5,
                                            Test1_point6, Test1_point7, Test1_point8, Test1_point9]
                    time_delay = 1
                    adjust_angles = True
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                    joint_angles_degrees = [home_position_closed, Pre_pour_position, Pour_position, Pre_pour_position,
                                            home_position_closed]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                    joint_angles_degrees = [Test1_point9, Test1_point8, Test1_point7, Test1_point6, Test1_point5,
                                            Test1_point4,
                                            Test1_point3, Test1_point2, Test1_point1, Test1_start_closed,
                                            Test1_start_open]
                    time_delay = 1
                    adjust_angles = True
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                    joint_angles_degrees = [Pre_pickup_tube1]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                elif group == 2:
                    # Execute action group 2
                    joint_angles_degrees = [Pre_pickup_tube2]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                    joint_angles_degrees = [Test_start_open, Test_start_closed, Test_point1, Test_point2, Test_point3,
                                            Test_point4, Test_point5,
                                            Test_point6, Test_point7, Test_point8, Test_point9]
                    time_delay = 1
                    adjust_angles = True
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                    joint_angles_degrees = [home_position_closed, Pre_pour_position, Pour_position, Pre_pour_position,
                                            home_position_closed]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                    joint_angles_degrees = [Test_point9, Test_point8, Test_point7, Test_point6, Test_point5,
                                            Test_point4,
                                            Test_point3, Test_point2, Test_point1, Test_start_closed, Test_start_open]
                    time_delay = 1
                    adjust_angles = True
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                    joint_angles_degrees = [Pre_pickup_tube2]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

                elif group == 3:
                    # Execute action group 3
                    joint_angles_degrees = [Pre_pickup_tube3]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)
                    # set nect position to lift tube
                    joint_angles_degrees = [Test3_start_open, Test3_start_closed, Test3_point1, Test3_point2,
                                            Test3_point3, Test3_point4,
                                            Test3_point5, Test3_point6, Test3_point7, Test3_point8, Test3_point9]
                    time_delay = 1
                    adjust_angles = True
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)
                    # setting position to pour tube
                    joint_angles_degrees = [home_position_closed, Pre_pour_position, Pour_position, Pre_pour_position,
                                            home_position_closed]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)
                    # setting position to replace tube
                    joint_angles_degrees = [Test3_point9, Test3_point8, Test3_point7, Test3_point6, Test3_point5,
                                            Test3_point4,
                                            Test3_point3, Test3_point2, Test3_point1, Test3_start_closed,
                                            Test3_start_open]
                    time_delay = 1
                    adjust_angles = True
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)
                    # going back to home
                    joint_angles_degrees = [Pre_pickup_tube2]
                    time_delay = 2
                    adjust_angles = False
                    execute_joint_angles(robot, Hiwonder, joint_angles_degrees, time_delay, adjust_angles)

        except ValueError as e:
            print(f"Error: {e}")
    else:
        print("Exiting Program Now.")
        break

