import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import cv2
import numpy as np
import math

from dr_writer import config
DRAWING_PAHT = config.DRAWING_PATH

# g_image_path = "doosan.png"
# g_image_path = "doosan.svg"
# g_image_path = "kirby.webp"
# g_image_path = "ryan.jpg"
# g_image_path = "square.png"
g_image_path = "test_flower_images.png"

def extract_lines_from_image(image_path, canvas_size=(400, 400)):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("이미지를 불러오지 못했습니다:", image_path)
        return []

    img_resized = cv2.resize(img, canvas_size, interpolation=cv2.INTER_AREA)
    edges = cv2.Canny(img_resized, 100, 200)

    lines = cv2.HoughLinesP(edges, 1, math.pi / 180, threshold=100, minLineLength=50, maxLineGap=10)

    path = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            path.append((x1, y1))
            path.append((x2, y2))
            path.append((-1, -1))
            
    return path
def extract_single_line_contours(img_gray, canvas_size=(400, 400)):
    img_resized = cv2.resize(img_gray, canvas_size)
    _, thresh = cv2.threshold(img_resized, 200, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = []
    for contour in contours:
        if cv2.arcLength(contour, True) < 100:  # 너무 작은 선은 제거
            continue
        for pt in contour:
            x, y = pt[0]
            result.append((x, y))
        result.append((-1, -1))
    return result

# def extract_curve_path_from_image(image_path, canvas_size=(400, 400)):
#     img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
#     if img is None:
#         print("이미지를 불러오지 못했습니다:", image_path)
#         return []

#     img_resized = cv2.resize(img, canvas_size, interpolation=cv2.INTER_AREA)
#     edges = cv2.Canny(img_resized, 100, 200)
#     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

#     path = []
#     for contour in contours:
#         for pt in contour:
#             x, y = pt[0]
#             path.append((x, y))
#         path.append((-1, -1))
#     return path

# def extract_curve_path_from_image(image_path, canvas_size=(400, 400)):
#     img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
#     if img is None:
#         print("이미지를 불러오지 못했습니다:", image_path)
#         return []

#     img_resized = cv2.resize(img, canvas_size, interpolation=cv2.INTER_AREA)

#     # Canny 대신 Threshold로 단일 윤곽선 검출
#     _, thresh = cv2.threshold(img_resized, 200, 255, cv2.THRESH_BINARY_INV)

#     contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

#     path = []
#     for contour in contours:
#         if cv2.arcLength(contour, True) < 100:  # 너무 짧은 곡선은 무시
#             continue
#         for pt in contour:
#             x, y = pt[0]
#             path.append((x, y))
#         path.append((-1, -1))
#     return path

# def extract_curve_path_from_image(image_path, canvas_size=(400, 400)):
#     import numpy as np
#     import cv2
#     import math

#     img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
#     if img is None:
#         print("이미지를 불러오지 못했습니다:", image_path)
#         return []

#     img_resized = cv2.resize(img, canvas_size, interpolation=cv2.INTER_AREA)
#     _, thresh = cv2.threshold(img_resized, 200, 255, cv2.THRESH_BINARY_INV)

#     contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

#     def angle_between(v1, v2):
#         unit_v1 = v1 / (np.linalg.norm(v1) + 1e-6)
#         unit_v2 = v2 / (np.linalg.norm(v2) + 1e-6)
#         dot_product = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
#         return np.arccos(dot_product) * 180 / math.pi

#     path = []
#     for contour in contours:
#         if len(contour) < 3 or cv2.arcLength(contour, True) < 100:
#             continue

#         prev_pt = contour[0][0]
#         path.append(tuple(prev_pt))

#         for i in range(1, len(contour) - 1):
#             curr_pt = contour[i][0]
#             next_pt = contour[i + 1][0]
            

#             # 벡터 계산
#             v1 = np.array(curr_pt) - np.array(prev_pt)
#             v2 = np.array(next_pt) - np.array(curr_pt)

#             angle = angle_between(v1, v2)

#             # 현재 점 추가
#             path.append(tuple(curr_pt))

#             # 각도가 급하게 꺾이면 -1, -1 삽입
#             if angle > 80:  # 80도 이상 꺾임으로 간주
#                 path.append((-1, -1))
#                 # 1879
#                 print("curr_pt: ", curr_pt, "type: ", type(curr_pt))
#                 print("prev_pt: ", prev_pt, "type: ", type(prev_pt))
#                 print("next_pt: ", next_pt, "type: ", type(next_pt))
#                 print("----------------")

#             prev_pt = curr_pt

#         # 마지막 점과 종결자
#         path.append(tuple(contour[-1][0]))
#         path.append((-1, -1))

#     return path

def extract_curve_path_from_image(image_path, canvas_size=(400, 400), step=5):
    import numpy as np
    import cv2
    import math

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("이미지를 불러오지 못했습니다:", image_path)
        return []

    img_resized = cv2.resize(img, canvas_size, interpolation=cv2.INTER_AREA)
    _, thresh = cv2.threshold(img_resized, 200, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    def calc_angle(v1, v2):
        unit_v1 = v1 / (np.linalg.norm(v1) + 1e-6)
        unit_v2 = v2 / (np.linalg.norm(v2) + 1e-6)
        dot = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
        return np.arccos(dot) * 180 / math.pi

    path = []
    for contour in contours:
        contour = [pt[0] for pt in contour]
        if len(contour) < step * 2 + 1 or cv2.arcLength(np.array(contour).reshape(-1, 1, 2), True) < 100:
            continue

        cooldown = 0
        i = 0
        while i < len(contour):
            path.append(tuple(contour[i]))

            if cooldown > 0:
                cooldown -= 1
                i += 1
                continue

            if i >= step and i + step < len(contour):
                prev_vec = np.mean(np.diff(contour[i - step:i], axis=0), axis=0)
                next_vec = np.mean(np.diff(contour[i + 1:i + 1 + step], axis=0), axis=0)

                angle = calc_angle(prev_vec, next_vec)

                if angle > 40:
                    path.append((-1, -1))
                    cooldown = step  # 스텝만큼 쉬고 다시 검사

            i += 1

        path.append((-1, -1))

    return path




def extract_circles_from_image(image_path, canvas_size=(400, 400)):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print("이미지를 불러오지 못했습니다:", image_path)
        return []

    img_resized = cv2.resize(img, canvas_size, interpolation=cv2.INTER_AREA)
    blurred = cv2.medianBlur(img_resized, 5)

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.0,
        minDist=10,
        param1=100,
        param2=20,         # ★ 감도 높임
        minRadius=3,       # ★ 작은 원 허용
        maxRadius=30
    )

    path = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            for angle in range(0, 360, 5):
                rad = np.deg2rad(angle)
                px = int(x + r * np.cos(rad))
                py = int(y + r * np.sin(rad))
                path.append((px, py))
            path.append((-1, -1))
    return path

def detect_circles_refined(img_gray, canvas_size=(400, 400)):
    import cv2
    import numpy as np

    # 리사이즈
    img_resized = cv2.resize(img_gray, canvas_size)
    blurred = cv2.medianBlur(img_resized, 5)

    # 원 검출: param2를 높이면 더 완전한 원만 검출됨
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=30,
        param1=100,
        param2=90,  # 높이면 완전한 원만 잡힘
        minRadius=10,
        maxRadius=80
    )

    result = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            for angle in range(0, 360, 5):
                rad = np.deg2rad(angle)
                px = int(x + r * np.cos(rad))
                py = int(y + r * np.sin(rad))
                result.append((px, py))
            result.append((-1, -1))
    return result



class DrawingNode(Node):
    def __init__(self):
        super().__init__('multi_stroke_drawing')
        self.publisher_ = self.create_publisher(Float32MultiArray, DRAWING_PAHT, 10)

        self.path = []
        self.is_drawing = False

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

        self.image_button = tk.Button(self.button_frame, text="Image to Path", command=lambda: self.load_and_publish_image_path(image_path=g_image_path))
        self.image_button.pack(side=tk.LEFT, padx=10)

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
        self.canvas.delete("all")
        self.path = []
        print("Canvas cleared.")

    def publish_path(self):
        if not self.path:
            print("No drawing to send!")
            return

        data = []
        for pt in self.path:
            # print("pt: ", pt)
            data.extend([float(pt[0] / 2), float(pt[1] / 2)])

        msg = Float32MultiArray()
        msg.data = data
        self.publisher_.publish(msg)
        print(f"Published path with {len(self.path)} points.")

    def draw_path_on_canvas(self):
        self.canvas.delete("all")
        prev = None
        for pt in self.path:
            if pt == (-1, -1):
                prev = None
            else:
                if prev is not None:
                    self.canvas.create_line(prev[0], prev[1], pt[0], pt[1], fill="blue")
                prev = pt

    # def load_and_publish_image_path(self, image_path):
    #     canvas_size = (400, 400)

    #     lines = extract_lines_from_image(image_path, canvas_size)
    #     # lines = extract_single_line_contours(image_path, canvas_size)
        
    #     curves = extract_curve_path_from_image(image_path, canvas_size)
    #     # circles = extract_circles_from_image(image_path, canvas_size)
    #     circles = detect_circles_refined(image_path, canvas_size)
        

    #     self.path = lines + curves + circles
    #     # self.path = curves
    #     self.draw_path_on_canvas()
    #     self.publish_path()
    def load_and_publish_image_path(self, image_path):
        canvas_size = (400, 400)

        # 이미지 한번만 읽기
        img_gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img_gray is None:
            print("이미지를 불러올 수 없습니다:", image_path)
            return

        # 각 추출 함수에 전달
        # lines = extract_single_line_contours(img_gray, canvas_size)
        lines = extract_single_line_contours(img_gray, canvas_size)
        curves = extract_curve_path_from_image(image_path, canvas_size)
        circles = detect_circles_refined(img_gray, canvas_size)

        self.path = lines + curves + circles
        # self.path = circles
        self.draw_path_on_canvas()
        self.publish_path()

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
