"""
part.gcode on a FAIRINO cobot - exported by FAIRINO G-code Runner 0.1.0 on 2026-10-17 08:12.
Frames          : tool 0, BASE frame
Posture         : fixed [180.0, 0.0, 0.0]
Instructions    : Circle x1, MoveC x2, MoveJ x4, MoveL x6
Path length     : 406.8 mm   (est. 0.5 min)
Frame bbox      : X 400.0..520.0  Y 50.0..110.0  Z 150.0..160.0 mm

The coordinates are fixed: if the workpiece or the frames change, export again.
Keep the emergency stop within reach. Ctrl-C stops the arm.
"""
import time
from fairino import Robot

TOOL, USER = 0, 0  # must be the frames active on the controller
ACC = 200.0        # mm/s^2 - lower it if a move fails with error 183

# Establish connection with robot controller
robot = Robot.RPC('192.168.58.2')


def joints_for(pose):
    """Joint angles for a Cartesian pose, closest to where the arm is now (MoveJ needs them)."""
    return robot.GetInverseKinRef(0, pose, robot.GetActualJointPosDegree()[1])[1]


# Refuse to run in the wrong frames: the coordinates below are expressed in them
if (robot.GetActualTCPNum()[1], robot.GetActualWObjNum()[1]) != (TOOL, USER):
    raise SystemExit(f"Activate tool {TOOL} / workpiece {USER} on the controller first.")
if input("The robot will MOVE. Type yes to start: ").strip().lower() != "yes":
    raise SystemExit("Cancelled.")

robot.Mode(0)                       # automatic mode
robot.RobotEnable(1)
robot.SetSpeed(100)                    # global speed %; G-code feed rates are only true at 100

try:
    # approach: 15 mm above the first point
    p = [400.0, 50.0, 170.0, 180.0, 0.0, 0.0]
    robot.MoveJ(joint_pos=joints_for(p), tool=TOOL, user=USER, desc_pos=p, vel=10.0, blendT=-1)
    # N3 G0 X0 Y0 Z5 S3000
    p = [400.0, 50.0, 155.0, 180.0, 0.0, 0.0]
    robot.MoveJ(joint_pos=joints_for(p), tool=TOOL, user=USER, desc_pos=p, vel=20.0, blendT=-1)
    # N4 M3
    robot.SetDO(2, 1)
    # N5 G1 Z0 F300
    robot.MoveL(desc_pos=[400.0, 50.0, 150.0, 180.0, 0.0, 0.0], tool=TOOL, user=USER,
                vel=100, ovl=5.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N6 G1 X40 Y0 F900
    robot.MoveL(desc_pos=[440.0, 50.0, 150.0, 180.0, 0.0, 0.0], tool=TOOL, user=USER,
                vel=100, ovl=15.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N7 G1 X40 Y30
    robot.MoveL(desc_pos=[440.0, 80.0, 150.0, 180.0, 0.0, 0.0], tool=TOOL, user=USER,
                vel=100, ovl=15.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N8 G2 X10 Y30 I-15 J0
    robot.MoveC(desc_pos_p=[425.0, 65.0, 150.0, 180.0, 0.0, 0.0], tool_p=TOOL, user_p=USER,
                desc_pos_t=[410.0, 80.0, 150.0, 180.0, 0.0, 0.0], tool_t=TOOL, user_t=USER,
                vel_p=100, vel_t=100, ovl=15.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N9 G1 X0 Y0
    robot.MoveL(desc_pos=[400.0, 50.0, 150.0, 180.0, 0.0, 0.0], tool=TOOL, user=USER,
                vel=100, ovl=15.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N10 G4 P0.2
    time.sleep(0.2)
    # N11 G0 Z5
    p = [400.0, 50.0, 155.0, 180.0, 0.0, 0.0]
    robot.MoveJ(joint_pos=joints_for(p), tool=TOOL, user=USER, desc_pos=p, vel=20.0, blendT=-1)
    # N12 G0 X80 Y20
    p = [480.0, 70.0, 155.0, 180.0, 0.0, 0.0]
    robot.MoveJ(joint_pos=joints_for(p), tool=TOOL, user=USER, desc_pos=p, vel=20.0, blendT=-1)
    # N13 G1 Z0 F300
    robot.MoveL(desc_pos=[480.0, 70.0, 150.0, 180.0, 0.0, 0.0], tool=TOOL, user=USER,
                vel=100, ovl=5.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N14 G2 I10 J0 F600
    robot.Circle(desc_pos_p=[490.0, 80.0, 150.0, 180.0, 0.0, 0.0], tool_p=TOOL, user_p=USER,
                 desc_pos_t=[500.0, 70.0, 150.0, 180.0, 0.0, 0.0], tool_t=TOOL, user_t=USER,
                 vel_p=100, vel_t=100, ovl=10.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N15 M0
    input('N15: paused - press Enter to continue ')
    # N16 G1 X100 Y20 F600
    robot.MoveL(desc_pos=[500.0, 70.0, 150.0, 180.0, 0.0, 0.0], tool=TOOL, user=USER,
                vel=100, ovl=10.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N17 G3 X100 Y60 R20
    robot.MoveC(desc_pos_p=[520.0, 90.0, 150.0, 180.0, 0.0, 0.0], tool_p=TOOL, user_p=USER,
                desc_pos_t=[500.0, 110.0, 150.0, 180.0, 0.0, 0.0], tool_t=TOOL, user_t=USER,
                vel_p=100, vel_t=100, ovl=10.0, oacc=ACC, blendR=-1, velAccParamMode=1)
    # N18 M5
    robot.SetDO(2, 0)
    # N19 G0 Z10
    p = [500.0, 110.0, 160.0, 180.0, 0.0, 0.0]
    robot.MoveJ(joint_pos=joints_for(p), tool=TOOL, user=USER, desc_pos=p, vel=20.0, blendT=-1)
    # retract: 15 mm above the last point
    p = [500.0, 110.0, 175.0, 180.0, 0.0, 0.0]
    robot.MoveJ(joint_pos=joints_for(p), tool=TOOL, user=USER, desc_pos=p, vel=20.0, blendT=-1)
    print("Done.")
except KeyboardInterrupt:
    robot.StopMotion()
    print("Stopped.")
finally:
    robot.CloseRPC()
