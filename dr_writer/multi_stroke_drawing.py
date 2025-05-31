import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk

from dr_writer import config
DRAWING_PAHT = config.DRAWING_PATH

class DrawingNode(Node):
    def __init__(self):
        super().__init__('multi_stroke_drawing')
        self.publisher_ = self.create_publisher(Float32MultiArray, DRAWING_PAHT, 10)

        self.path = []
        self.is_drawing = False

        # Tkinter GUI
        self.root = tk.Tk()
        self.root.title("ROS2 Drawing Node")
        self.canvas = tk.Canvas(self.root, width=400, height=400, bg="white")
        self.canvas.pack()
        self.canvas.bind("<ButtonPress-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.end_draw)

        self.button_frame = tk.Frame(self.root)
        self.button_frame.pack()

        self.finish_button = tk.Button(self.button_frame, text="Send Path", command=self.publish_path)
        self.finish_button.pack(side=tk.LEFT, padx=10)

        self.clear_button = tk.Button(self.button_frame, text="Clear", command=self.clear_canvas)
        self.clear_button.pack(side=tk.LEFT, padx=10)

    def start_draw(self, event):
        self.is_drawing = True
        self.path.append((event.x, event.y))

    def draw(self, event):
        if self.is_drawing:
            self.canvas.create_line(self.path[-1][0], self.path[-1][1], event.x, event.y)
            self.path.append((event.x, event.y))

    def end_draw(self, event):
        if self.is_drawing:
            self.path.append((event.x, event.y))
            self.path.append((-1, -1))
            self.is_drawing = False

    def clear_canvas(self):
        self.canvas.delete("all")  # 화면 지우기
        self.path = []             # 경로도 초기화
        print("Canvas cleared.")

    def publish_path(self):
        if not self.path:
            print("No drawing to send!")
            return
        
        print(self.path)
        
        # 2D path를 1D 배열로 평탄화해서 전송: [x0, y0, x1, y1, ...]
        data = []
        for pt in self.path:
            data.extend([float(pt[0] / 2), float(pt[1] / 2)])

        msg = Float32MultiArray()
        msg.data = data
        self.publisher_.publish(msg)
        print(f"Published path with {len(self.path)} points.")

        # 캔버스와 경로 데이터 초기화
        # self.canvas.delete("all")  # 화면 지우기
        # self.path = []             # 경로도 초기화
        self.clear_canvas()

    def spin(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = DrawingNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
ros2 topic pub --once /drawing_path std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, 200.0, 0.0, 200.0, 200.0, 0.0, 200.0, 0.0, 0.0]}"
"""