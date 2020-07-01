import glob, os, math
from Phidget22.Phidget import *
from Phidget22.Devices.Stepper import *

# file name for an 'erase' trajectory that clears sand, runs the ball in a spiral
ERASE_THR = 'erase.thr'

# acceleration and velocity bounds on stepper motors
ACC = 1
MAX_VEL = 3

# conversion factors to get the stepper coordinates in degrees and inches
# degrees per 1 step of motor
STEP_ANGLE = 1.8
# ratio between stepper motor and large pulley for R joint
R_JOINT_RATIO = 20
# 16*pi/25.4 in circumference pulley, 16pi/25.4/360 in per 1 degree 
DEG_PER_IN_P = 1 / (16 * math.pi / 25.4 / 360)
# length of the prismatic joint from its calibration point in inches
# defines radius of workspace
JOINT_LENGTH_P = 400 / 25.4

# current target position of R joint, in degrees
curr_target_pos_r = 0
# current target position of P joint, in inches
curr_target_pos_p = 0

# tolerances for how close the end effector is for marking as 
# 'arrived' at target positions
R_POS_TOLERANCE = 0.1
P_POS_TOLERANCE = 0.1

def init_arm():
    stepper_r = Stepper()
    stepper_r.setHubPort(0)
    stepper_p = Stepper()
    stepper_p.setHubPort(1)
    stepper_r.openWaitForAttachment(5000)
    stepper_p.openWaitForAttachment(5000)
    stepper_r.setRescaleFactor((1/16) * (STEP_ANGLE/R_JOINT_RATIO))
    stepper_p.setRescaleFactor((1/16) * (STEP_ANGLE/DEG_PER_IN_P))
    stepper_r.setAcceleration(ACC)
    stepper_p.setAcceleration(ACC)
    stepper_r.setVelocityLimit(MAX_VEL)
    stepper_p.setVelocityLimit(MAX_VEL)
    stepper_r.setEngaged(True)
    stepper_p.setEngaged(True)

    # Drive P joint until limit switch is hit to find 0 coordinate
    calib_pos = 0
    offset_pos = None
    while calib_pos < JOINT_LENGTH_P and offset_pos is None:
        calib_pos += 0.25
        stepper_p.setTargetPosition(calib_pos)
        while stepper_p.getIsMoving():
            if limit_switch_hit:
                offset_pos = stepper_p.getPosition()
                break
    stepper_p.addPositionOffset(offset_pos)
    stepper_p.setTargetPosition(0)    
    
def move_arm_to_position(theta, rho):
    curr_target_pos_r = theta / (2 * math.pi/360)
    curr_target_pos_p = rho * JOINT_LENGTH_P
    stepper_r.setTargetPosition(CURR_TARGET_POS_R)
    stepper_p.setTargetPosition(CURR_TARGET_POS_P)
    r_pos_diff = None
    p_pos_diff = None
    # loop until we arrive at the target position
    while (r_pos_diff is None and p_pos_diff is None) or (r_pos_diff > R_POS_TOLERANCE and p_pos_diff > P_POS_TOLERANCE):
        curr_r_pos = stepper_r.getPosition()
        curr_p_pos = stepper_p.getPosition()
        r_pos_diff = abs(curr_target_pos_r - curr_r_pos)
        p_pos_diff = abs(curr_target_pos_p - curr_p_pos)
    
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
    os.chdir("/../theta_rho")
    # just loop through the files in the thr directory in a random shuffled order
    while(True):
        thr_files = glob.glob('*.thr')
        random.shuffle(thr_files)
        for thr_file in thr_files:
            print(thr_file)
            follow_theta_rho_trajectory(thr_file)
            follow_theta_rho_trajectory(ERASE_THR)
    
    
