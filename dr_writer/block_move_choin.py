import rclpy
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
JReady = [0, 0, 90, 0, 90, 0]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT2 import (
    movej, movel, amovel,
    task_compliance_ctrl, set_desired_force,
    release_compliance_ctrl, release_force,
    check_force_condition, get_current_posx,
    set_digital_output, get_digital_input, wait,
    set_tool, set_tcp,
    DR_AXIS_Z, DR_BASE, DR_FC_MOD_REL, DR_MV_MOD_REL
) 
from DR_common2 import posx

ON, OFF = 1, 0

# 블록 원래 위치 (org) 및 배치 위치 (dst)
orgs = [0,
    posx(499.0, 145.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(552.0, 145.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(600.0, 145.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(499.0,  95.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(550.0,  95.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(600.0,  95.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(499.0,  45.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(550.0,  45.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(600.0,  45.0, 130.0, 0.0, 0.0, 180.0, 0.0),
]

dsts = [0,
    posx(499.0, -56.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(550.0, -56.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(600.0, -56.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(499.0, -106.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(550.0, -106.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(600.0, -106.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(499.0, -159.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(545.0, -159.0, 130.0, 0.0, 0.0, 180.0, 0.0),
    posx(600.0, -159.0, 130.0, 0.0, 0.0, 180.0, 0.0),
]

def return_position_org(i): return orgs[i]
def return_position_dst(i): return dsts[i]

def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)
        print("Wait for digital input...")

def open_gripper():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait_digital_input(2)

def close_gripper():
    open_gripper()  # 먼저 열기 (안전)
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait_digital_input(1)

def move_block_with_height_detection(start_idx, target_idx):
    start_pos = return_position_org(start_idx)
    end_pos = return_position_dst(target_idx)

    # 이동 및 높이 감지
    movej(JReady, vel=VELOCITY, acc=ACC)
    movel(start_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    task_compliance_ctrl([3000, 3000, 0, 200, 200, 200])
    set_desired_force([0, 0, -30, 0, 0, 0], [0, 0, 1, 0, 0, 0], DR_FC_MOD_REL)

    start_z = get_current_posx()[2]
    while not check_force_condition(DR_AXIS_Z, min=15):
        time.sleep(0.01)
    end_z = get_current_posx()[2]
    z_diff = start_z - end_z

    release_force()
    release_compliance_ctrl()

    # 높이 판단
    if z_diff < 10:
        height = 1
    elif z_diff < 20:
        height = 2
    else:
        height = 3

    print(f"감지된 높이: {height}단 (Z 이동: {z_diff:.2f} mm)")

    # Pick
    drop_rel = posx(0, 0, -z_diff, 0, 0, 0)
    up_rel = posx(0, 0, z_diff + 30, 0, 0, 0)

    movel(drop_rel, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    close_gripper()
    time.sleep(0.2)
    movel(up_rel, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    # Place
    movel(end_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    movel(drop_rel, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    open_gripper()
    time.sleep(0.2)
    movel(up_rel, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    print(f"[완료] 블록 이동: {start_idx} → {target_idx}, 높이: {height}단")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("block_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
    movej, movel, amovel,
    task_compliance_ctrl, set_desired_force,
    release_compliance_ctrl, release_force,
    check_force_condition, get_current_posx,
    set_digital_output, get_digital_input, wait,
    set_tool, set_tcp,
    DR_AXIS_Z, DR_BASE, DR_FC_MOD_REL, DR_MV_MOD_REL
)
    from DR_common2 import posx

    set_tool("Tool Weight_3_24")
    set_tcp("TCP208mm")

    for i in range(1, 10):
        move_block_with_height_detection(i, i)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
