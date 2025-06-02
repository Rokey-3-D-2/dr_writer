import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import cv2
import numpy as np
import matplotlib.pyplot as plt

from dr_writer import config
DRAWING_PAHT = config.DRAWING_PATH

class EdgeExtractorNode(Node):
    def __init__(self):
        super().__init__('edge_extractor_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, DRAWING_PAHT, 10)
        self.process_image()

    def process_image(self):
        # 이미지 읽기 & 엣지 추출
        img = cv2.imread("/home/lhj/Downloads/test_flower_images.png", cv2.IMREAD_GRAYSCALE)
        
        h, w = img.shape
        scale = 200 / max(h, w)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        # 200x200 흰 canvas를 만들고, 왼쪽 위(0,0)에 붙임 (padding 없음, 남는 부분은 그냥 배경)
        img_canvas = np.ones((200, 200), dtype=np.uint8) * 255
        img_canvas[0:new_h, 0:new_w] = resized
        img = img_canvas    

        # 1. Blur
        img_blur = cv2.GaussianBlur(img, (3,3), 0)
        # 2. Adaptive Threshold로 선 강조
        bin_img = cv2.adaptiveThreshold(img_blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                cv2.THRESH_BINARY_INV, 11, 2)
        # 3. Morphology로 노이즈/끊김 개선
        kernel = np.ones((3,3), np.uint8)
        morph_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel)
        # 4. Canny로 엣지 따기
        edges = cv2.Canny(morph_img, 70, 180)
        # contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # 1. 전체 컨투어 좌표 합치기
        all_pts = np.vstack([c.squeeze() for c in contours if len(c) >= 10])
        # 2. 전체 bounding box 구하기
        min_xy = np.min(all_pts, axis=0)
        max_xy = np.max(all_pts, axis=0)
        size = max_xy - min_xy
        scale = 200.0 / size.max()
        # 3. 모든 컨투어를 "전체 bbox 기준"으로 변환
        all_points = []
        plt.figure(figsize=(6, 6))
        plt.title("All Contours - Relative Position Preserved\n-1,-1 for Pen Up")
        plt.xlim(0, 200)
        plt.ylim(0, 200)
        plt.gca().set_aspect('equal')
        plt.gca().invert_yaxis()

        for i, contour in enumerate(contours):
            if len(contour) < 10:
                continue
            contour = contour.squeeze()
            if len(contour.shape) == 1:
                continue
            contour_scaled = (contour - min_xy) * scale
            # 전체 bbox 기준 중앙 정렬
            min_s = np.min((all_pts - min_xy) * scale, axis=0)
            max_s = np.max((all_pts - min_xy) * scale, axis=0)
            offset = (200 - (max_s - min_s)) / 2 - min_s
            contour_canvas = contour_scaled + offset
            # # 샘플링
            # max_points = 127
            # n = len(contour_canvas)
            # if n > max_points:
            #     idx = np.linspace(0, n-1, max_points, dtype=int)
            #     contour_canvas = contour_canvas[idx]
            for pt in contour_canvas:
                all_points.append([pt[0], pt[1]])
            all_points.append([-1, -1])
            plt.plot(contour_canvas[:, 0], contour_canvas[:, 1], marker='o', markersize=2, linewidth=1, label=f'Contour {i+1}')

        plt.legend()
        plt.show()

        self.get_logger().info(f"경계 포함 전체 path 좌표 수: {len(all_points)}")
        self.get_logger().info(f"일부 샘플: {all_points[:10]}")

        # Float32MultiArray로 변환
        data = []
        for pt in all_points:
            data.extend([float(pt[0]), float(pt[1])])
        msg = Float32MultiArray()
        msg.data = data
        self.publisher_.publish(msg)
        # self.get_logger().info(f"총 {len(all_points)}개 경로 publish 완료")

def main(args=None):
    rclpy.init(args=args)
    node = EdgeExtractorNode()
    rclpy.spin_once(node, timeout_sec=0.1)  # 1회 실행 후 종료 (필요시 spin으로 반복)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
