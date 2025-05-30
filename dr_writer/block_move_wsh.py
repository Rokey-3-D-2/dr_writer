# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            release_force,
            DR_TOOL,
        )
        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    # ========== grip ==========
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            time.sleep(0.5)
            print("Wait for digital input")
            pass

    def release():
        print("release")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        # wait_digital_input(2)

    def grip():
        print("grip")
        # release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        # wait_digital_input(1)
    # ========== ==========
    pos_org_1 = posx([499.0, 145.0, 130.0, 0.0, 180.0, 0.0])
    pos_org_2 = posx([552.0, 145.0, 130.0, 0.0, 180.0, 0.0])
    pos_org_3 = posx([600.0, 145.0, 130.0, 0.0, 180.0, 0.0])
    pos_org_4 = posx([499.0, 95.0, 130.0, 0.0, 180.0, 0.0])
    pos_org_5 = posx([550.0, 95.0, 130.0, 0.0, 180.0, 0.0])
    pos_org_6 = posx([600.0, 95.0, 130.0, 0.0, 180.0, 0.0])
    pos_org_7 = posx([499.0, 45.0, 130.0, 0.0, 180.0, 0.0])
    pos_org_8 = posx([550.0, 45.0, 130.0, 0.0, 180.0, 0.0])
    pos_org_9 = posx([600.0, 45.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_1 = posx([499.0, -56.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_2 = posx([550.0, -56.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_3 = posx([600.0, -56.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_4 = posx([499.0, -106.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_5 = posx([550.0, -106.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_6 = posx([600.0, -106.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_7 = posx([499.0, -159.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_8 = posx([545.0, -159.0, 130.0, 0.0, 180.0, 0.0])
    pos_dst_9 = posx([600.0, -159.0, 130.0, 0.0, 180.0, 0.0])

    pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])



    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    set_tool("TCP208mm")
    set_tcp("Tool Weight_3_24")
    
    # def move_block(start, end, height):
        

    while rclpy.ok():
        # 초기 위치로 이동
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        print("input start: ", end="")
        start_val = int(input())
        print(start_val)

        print("input end: ", end="")
        end_val = int(input())
        print(end_val)

        start_pos = 0
        if start_val==1:
            print("pos = pos_org_1")
            start_pos = pos_org_1
        elif start_val ==2:
            start_pos = pos_org_2
        elif start_val ==3:
            start_pos = pos_org_3
        elif start_val ==4:
            start_pos = pos_org_4
        elif start_val ==5:
            start_pos = pos_org_5
        elif start_val ==6:
            start_pos = pos_org_6
        elif start_val ==7:
            start_pos = pos_org_7
        elif start_val ==8:
            start_pos = pos_org_8
        elif start_val ==9:
            start_pos = pos_org_9
        
        end_pos = 0
        if end_val==1:
            end_pos = pos_dst_1
        elif end_val ==2:
            end_pos = pos_dst_2
        elif end_val ==3:
            end_pos = pos_dst_3
        elif end_val ==4:
            end_pos = pos_dst_4
        elif end_val ==5:
            end_pos = pos_dst_5
        elif end_val ==6:
            end_pos = pos_dst_6
        elif end_val ==7:
            end_pos = pos_dst_7
        elif end_val ==8:
            end_pos = pos_dst_8
        elif end_val ==9:
            end_pos = pos_dst_9
        
        up_pose = posx([0, 0, -100, 0, 0, 0])
        down_pose = posx([0, 0, 120, 0, 0, 0])
        
        def move_up():
            print("move up")
            movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        def move_down():
            print("move down")
            movel(down_pose, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        
        def force_control_fun():
            print("force_control_start")
            
            task_compliance_ctrl(stx=[100, 100, 100, 100, 100, 100])
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            time.sleep(0.1)
        
            
            # # for real
            # while not check_force_condition(DR_AXIS_Z, max=5):
            #     print("working")
            #     pass
            # for virtual
            while not check_force_condition(DR_AXIS_Z, min=5):
                print("working")
                pass
            
            time.sleep(0.1)

            release_force()
            release_compliance_ctrl()
            
            print("force_control_end")
        
        # ========== 움직이기 시작 ==========
        if start_val== 0:
            break
        if end_val == 0:
            break
        
        
        # 시작 위치 이동
        grip()
        print("move start")
        movel(pos_org_1, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 힘 제어
        force_control_fun()
        
        # 올린 후 잡아서 들기
        move_up()
        release()
        move_down()
        grip()
        move_up()
        
        
        time.sleep(2)
        
        # 종료 위치 이동
        print("move end")
        movel(end_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        move_down()
        release()
        

    rclpy.shutdown()


if __name__ == "__main__":
    print()
    main()