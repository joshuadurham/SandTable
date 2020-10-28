import glob, os, math, random
from Phidget22.Phidget import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.DigitalInput import *
# file name for an 'erase' trajectory that clears sand, runs the ball in a spiral
ERASE_THR = 'erase.thr'

# acceleration and velocity bounds on stepper motors
ACC = 3.0
MAX_VEL = 8.0

# conversion factors to get the stepper coordinates in degrees and inches
# degrees per 1 step of motor
STEP_ANGLE = 1.8
# ratio between stepper motor and large pulley for R joint
R_JOINT_RATIO = 25
# 16*pi/25.4 in circumference pulley, 16pi/25.4/360 in per 1 degree 
DEG_PER_IN_P = 1 / (12.77 * math.pi / 25.4 / 360)
# length of the prismatic joint from its calibration point in inches
# defines radius of workspace
JOINT_LENGTH_P = 9

DIST_SWITCH_TO_CENTER = 4.8

# current target position of R joint, in degrees
curr_target_pos_r = 0
# current target position of P joint, in inches
curr_target_pos_p = 0

# tolerances for how close the end effector is for marking as 
# 'arrived' at target positions
R_POS_TOLERANCE = 0.1
P_POS_TOLERANCE = 0.1

def init_arm():
    global stepper_r
    stepper_r = Stepper()
    stepper_r.setHubPort(0)
    global stepper_p
    stepper_p = Stepper()
    stepper_p.setHubPort(1)
    global limit_switch
    limit_switch = DigitalInput()
    limit_switch.setIsHubPortDevice(True)
    limit_switch.setHubPort(2)
    stepper_r.openWaitForAttachment(5000)
    stepper_p.openWaitForAttachment(5000)
    limit_switch.openWaitForAttachment(5000)
    stepper_r.setCurrentLimit(2.0)
    stepper_r.setRescaleFactor((1.0/16.0) * (STEP_ANGLE/R_JOINT_RATIO))
    stepper_p.setRescaleFactor((1.0/16.0) * (STEP_ANGLE/DEG_PER_IN_P))
    stepper_r.setAcceleration(ACC)
    stepper_p.setAcceleration(ACC)
    stepper_r.setVelocityLimit(MAX_VEL)
    stepper_p.setVelocityLimit(MAX_VEL)
    stepper_r.setEngaged(True)
    stepper_p.setEngaged(True)

    # Drive P joint until limit switch is hit to find 0 coordinate
    calib_pos = 0
    offset_pos = None
    while calib_pos < 2*JOINT_LENGTH_P and offset_pos is None:
        calib_pos += 0.25
        stepper_p.setTargetPosition(-calib_pos)
        while stepper_p.getIsMoving():
            if limit_switch.getState():
                offset_pos = stepper_p.getPosition()
                break
    print(offset_pos)
    stepper_p.setTargetPosition(offset_pos + DIST_SWITCH_TO_CENTER)
    while stepper_p.getIsMoving():
        print('target: ' +  str(offset_pos + DIST_SWITCH_TO_CENTER))
        print('current: ' + str(stepper_p.getPosition()))
        print("centering")
    stepper_p.addPositionOffset(-(offset_pos + DIST_SWITCH_TO_CENTER))
    
def move_arm_to_position(theta, rho):
    curr_target_pos_r = theta / (2 * math.pi/360)
    curr_target_pos_p = rho * JOINT_LENGTH_P
    curr_r_pos = stepper_r.getPosition()
    curr_p_pos = stepper_p.getPosition()

    # Adjust the max velocity of each joint in order to syncronize their motion and 
    # have both joints reach their target positions at approximately same time
    r_time_to_goal = abs(curr_target_pos_r - curr_r_pos) / MAX_VEL
    p_time_to_goal = abs(curr_target_pos_p - curr_p_pos) / MAX_VEL

    if r_time_to_goal > p_time_to_goal and r_time_to_goal != 0:
        stepper_p.setVelocityLimit(MAX_VEL * (p_time_to_goal / r_time_to_goal))
    elif p_time_to_goal != 0:
        stepper_r.setVelocityLimit(MAX_VEL * (r_time_to_goal / p_time_to_goal))

    stepper_r.setTargetPosition(curr_target_pos_r)
    stepper_p.setTargetPosition(curr_target_pos_p)
    print(curr_target_pos_p)
    print(curr_target_pos_r)
    r_pos_diff = None
    p_pos_diff = None
    # loop until we arrive at the target position
    while (r_pos_diff is None and p_pos_diff is None) or (r_pos_diff > R_POS_TOLERANCE or p_pos_diff > P_POS_TOLERANCE):
        curr_r_pos = stepper_r.getPosition()
        curr_p_pos = stepper_p.getPosition()
        r_pos_diff = abs(curr_target_pos_r - curr_r_pos)
        p_pos_diff = abs(curr_target_pos_p - curr_p_pos)
    
    stepper_p.setVelocityLimit(MAX_VEL)
    stepper_r.setVelocityLimit(MAX_VEL)
    
def get_float(float_str):
    try:
        ret_val = float(float_str)
        return ret_val
    except ValueError:
        return None

def follow_theta_rho_trajectory(thr_file):
    f = open(thr_file, 'r')
    for line in f:
        print(line)
        line_toks = line.split()
        # first two tokens on a line must be valid floats
        if len(line_toks) >= 2:
            theta_val = get_float(line_toks[0])
            rho_val = get_float(line_toks[1])
            if theta_val is not None and rho_val is not None:
                move_arm_to_position(theta_val, rho_val)
    f.close()

def run_arm():
    init_arm()
    os.chdir("/home/pi/SandTable/theta_rho")
    # just loop through the files in the thr directory in a random shuffled order
    follow_theta_rho_trajectory(ERASE_THR)
    while(True):
        thr_files = glob.glob('*.thr')
        random.shuffle(thr_files)
        for thr_file in thr_files:
            print(thr_file)
            follow_theta_rho_trajectory(thr_file)
            follow_theta_rho_trajectory(ERASE_THR)
    
    
