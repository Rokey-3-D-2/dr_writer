import rclpy
import DR_init
import time

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL = config.VEL 
ACC = config.ACC
DRAWING_PAHT = config.DRAWING_PATH

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("multi_stroke_board", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        get_tcp, get_tool,
        set_tool, set_tcp, set_ref_coord,
        get_current_velx, 
        DR_WHITE_BOARD2,
        DR_AXIS_Z,
        DR_FC_MOD_REL
    )

    tcp, tool = get_tcp(), get_tool()
    print(f'tcp: {tcp}, tool: {tool}')
    if tcp != ROBOT_TCP or tool != ROBOT_TOOL:
        node.destroy_node()
        rclpy.shutdown()
        return

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_ref_coord(DR_WHITE_BOARD2)

    try:
        while True:
            node.get_logger().info(f'velx: {get_current_velx()}')
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()