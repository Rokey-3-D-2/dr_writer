import rclpy
import DR_init
from std_msgs.msg import Float32MultiArray
import time
import queue

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL, ACC = 30, 30

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

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("movel_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DR_common2 import posx, posj
    from DSR_ROBOT2 import movel, movej, set_tcp, set_tool

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

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # ROS2 콜백 처리(비동기, 빠른 주기)
            if not move_queue.empty():
                points = move_queue.get()
                for pt in points:
                    p = posx([pt[0], pt[1], 300, 0, 180, 0])
                    print(f"[Robot] Moving to: {p}")
                    movel(p, vel=VEL, acc=ACC)
                movej(home, vel=VEL, acc=ACC)
                print('[Robot] come back home')
    except KeyboardInterrupt:
        print('Shutting down...')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
