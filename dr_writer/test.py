import rclpy
import DR_init

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("movel_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DR_common2 import posx
    from DSR_ROBOT2 import movel, set_tcp, set_tool

    x1 = posx([500, 200, 300, 90, 0, 0])
    x2 = posx([300, 500, 200, 0, -90, 0])
    x3 = posx([200, 300, 500, 0, 0, 0])

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    if rclpy.ok():
        movel(x1, vel=VEL, acc=ACC)
        movel(x2, vel=VEL, acc=ACC)
        movel(x3, vel=VEL, acc=ACC)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()