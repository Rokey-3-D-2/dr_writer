import rclpy
import DR_init
from std_msgs.msg import Float32MultiArray
import numpy as np
import queue
import time

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL = config.VEL 
ACC = config.ACC

# 전역 큐
move_queue = queue.Queue()

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def listener_callback(msg):
    data = msg.data
    if len(data) % 2 != 0:
        print('길이가 2의 배수가 아닌 데이터를 받았습니다!')
        return
    points = []
    for i in range(0, len(data), 2):
        points.append([data[i], data[i+1]])
    # print(f"[Subscriber] 복원된 좌표 리스트: {points}")
    move_queue.put(points)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("white_board_test", namespace=ROBOT_ID)
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
        get_tool_force,
        get_joint_torque,
        
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

    node.create_subscription(
        Float32MultiArray,
        '/drawing_path',
        listener_callback,
        10
    )

    white_board_home = posx([0, 0, 0, 0, 0, 0])

    def sample_points(points, max_middle=20):
        """
        - points: (N, 2) 형태의 list 또는 np.ndarray
        - 시작점, 끝점은 반드시 포함
        - 중간 경유점은 10%만 균등간격 샘플링, 최대 max_middle개
        - 반환값: (샘플링된 경로 list)
        """
        n = len(points)
        if n <= 2:
            # 점이 2개 이하면 그대로 반환
            return points
        
        # 중간점 개수 계산
        middle_points = points[1:-1]
        num_middle = min(max_middle, max(1, int((n - 2) * 0.1)))
        if num_middle < 1:
            # 중간 경유점이 아예 없거나 너무 적음
            return points
        
        idx = np.linspace(0, len(middle_points)-1, num_middle, dtype=int)
        sampled_middle = [middle_points[i] for i in idx]
        sampled = np.vstack([points[0], sampled_middle, points[-1]])
        return sampled.tolist()
    
    def convert_to_posx(sampled_points):
        return [posx([pt[0], pt[1], 0, 0, 0, 0]) for pt in sampled_points]

    def release():
        release_compliance_ctrl()
        release_force()

    def pen_down():
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, 20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.1)

    def pen_up():
        release()

        traj[-1][2] -= 10
        movel(traj[-1], VEL, ACC)

    def draw_on_board(traj):
        ret = amovesx(traj, vel=VEL, acc=ACC)
        node.get_logger().info(f'after amovesx: {ret}')

    try:
        movel(white_board_home, VEL, ACC)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # ROS2 콜백 처리(비동기, 빠른 주기)
            if not move_queue.empty():
                points = move_queue.get()

                # sampled_points = sample_points(points, max_middle=20)
                # traj = convert_to_posx(sampled_points)
                traj = convert_to_posx(points)
                node.get_logger().info(f"Splined path 실행: {traj}, {len(traj)}")
                
                movel(traj[0], VEL, ACC)
                node.get_logger().info('move to start point')

                pen_down()
                node.get_logger().info('pen_down')

                while check_force_condition(DR_AXIS_Z, min=5, max=21): pass
                node.get_logger().info(f'before force = {get_joint_torque()}')
                release_force()
                set_desired_force(fd=[0, 0, 5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                node.get_logger().info(f'after force = {get_joint_torque()}')
                node.get_logger().info('touch on board!')

                # 스플라인 명령으로 경로 전체를 연속 실행
                node.get_logger().info('drawing')
                draw_on_board(traj)

                node.get_logger().info('waiting until drawing is done')
                # while True:
                #     mt = check_motion()
                #     node.get_logger().info(f'check motion : {mt}')
                #     if mt == 0:
                #         break
                # while check_motion() != 0: pass
                start = time.time()
                while check_motion() == 2: 
                    if time.time() - start > 30:
                        node.get_logger().warn(f'amovesx time out')
                        break
                node.get_logger().info('done')

                pen_up()
                node.get_logger().info('pen_up')
        
                movel(white_board_home, VEL, ACC)
                node.get_logger().info('comeback to origin')
    except KeyboardInterrupt:
        release()
        print('Shutting down...')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
