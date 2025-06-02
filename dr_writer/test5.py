import rclpy
import DR_init
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL = config.VEL 
ACC = config.ACC

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("amovesx_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DR_common2 import posx, posj
    from DSR_ROBOT2 import (
        get_tcp, get_tool,
        set_tcp, set_tool, 
        set_ref_coord,
        
        movej, movel, movesx, amovesx,
        check_motion, mwait,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,
        
        set_user_cart_coord,
        get_user_cart_coord,

        DR_WHITE_BOARD,
        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

    print(f'tcp: {get_tcp()}, tool: {get_tool()}')

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_ref_coord(DR_WHITE_BOARD)

    def wait_for_motion_done(timeout=30):
        """모션이 종료될 때까지 대기. timeout 초과 시 False 반환."""
        start = time.time()
        while True:
            done = check_motion()  # 0: 동작 없음(정상 종료), 1: 동작 중
            if done == 0:
                print("[SUCCESS] Motion finished.")
                return True
            if time.time() - start > timeout:
                print("[TIMEOUT] Motion did not finish within time.")
                return False
            time.sleep(0.5)

    def test_amovesx():
        # 테스트용 Spline Task 경로 3개 posx (실제 로봇 안전 구간 내로 설정!)
        p1 = posx(200, 0, 0, 0, 0, 0)
        p2 = posx(0, 200, 0, 0, 0, 0)
        p3 = posx(0, 0, -200, 0, 0, 0)
        path = [p1, p2, p3]

        # amovesx 비동기 실행
        print("amovesx 실행")
        ret = amovesx(path, vel=100, acc=200)
        if ret != 0:
            print(f"[FAIL] amovesx 호출 실패(ret={ret})")
            return

        # 즉시 반환되는지 확인
        print("amovesx 호출 즉시 반환됨. 실제 모션 종료까지 대기...")
        # check_motion으로 실제 동작 종료될 때까지 대기
        finished = wait_for_motion_done(timeout=60)
        if finished:
            print("[OK] amovesx 정상 종료 확인")
        else:
            print("[FAIL] amovesx 종료 안됨 (타임아웃 또는 오류)")

    white_board_home = posx([0, 0, 0, 0, 0, 0])
    
    movel(white_board_home, VEL, ACC)    
    test_amovesx()
    movel(white_board_home, VEL, ACC)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
