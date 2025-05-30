import rclpy
import DR_init
from std_msgs.msg import Float32MultiArray
import numpy as np
import queue

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL, ACC = 100, 100

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
    print(f"[Subscriber] 복원된 좌표 리스트: {points}")
    move_queue.put(points)

def sample_points(points, max_middle=20):
    # n = len(points)
    # if n <= max_points:
    #     return points
    # idx = np.linspace(0, n-1, max_points, dtype=int)
    # return [points[i] for i in idx]
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

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("movesx_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DR_common2 import posx, posj
    from DSR_ROBOT2 import (
        movej, set_tcp, set_tool, movesx, 
        get_current_posx
    )

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    home = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    if rclpy.ok():
        movej(home, vel=VEL, acc=ACC)

    node.create_subscription(
        Float32MultiArray,
        '/drawing_path',
        listener_callback,
        10
    )

    current_posx_z = get_current_posx()[0][2]
    print(current_posx_z)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # ROS2 콜백 처리(비동기, 빠른 주기)
            if not move_queue.empty():
                points = move_queue.get()

                sampled_points = sample_points(points, max_middle=20)
                traj = [posx([pt[0], pt[1], current_posx_z, 0, 180, 0]) for pt in sampled_points]

                # posx 리스트로 변환 (z/w/p/r 고정)
                # traj = []
                # for pt in points:
                #     traj.append(posx([pt[0], pt[1], 300, 0, 180, 0]))  # 필요시 z/w/p/r 조정

                # sampled_traj = sample_points(traj, max_middle=20)
                print(f"[Robot] Splined path 실행: {traj}, {len(traj)}")

                # 스플라인 명령으로 경로 전체를 연속 실행
                movesx(traj, vel=VEL, acc=ACC)

                movej(home, vel=VEL, acc=ACC)
                print('[Robot] come back home')
    except KeyboardInterrupt:
        print('Shutting down...')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
