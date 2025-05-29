import rclpy
import DR_init
from std_msgs.msg import Float32MultiArray

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("movel_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DR_common2 import posx, posj
    from DSR_ROBOT2 import movel, movej, set_tcp, set_tool, amovel

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    home = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    if rclpy.ok(): movej(home, vel=VEL, acc=ACC)
    
    def listener_callback(msg):
        data = msg.data
        
        if len(data) % 2 != 0:
            node.get_logger().error('길이가 2의 배수가 아닌 데이터를 받았습니다!')
            return
        
        points = []
        for i in range(0, len(data), 2):
            points.append([data[i], data[i+1]])
        node.get_logger().info(f"복원된 좌표 리스트: {points}")

        # ====== 예시: Z, 자세(오리엔테이션)는 고정, X/Y만 따라가기 ======
        for pt in points:
            # 예시) posx(x, y, z, w, p, r)
            p = posx([pt[0], pt[1], 300, 0, 0, 0])
            node.get_logger().info(f"Moving to: {p}")
            movel(p, vel=VEL, acc=ACC)

        movej(home, vel=VEL, acc=ACC)
        print('come back home')

    node.create_subscription(
        Float32MultiArray,
        '/drawing_path',
        listener_callback,
        10
    )

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()