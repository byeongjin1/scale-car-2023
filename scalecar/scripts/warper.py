import cv2
import numpy as np


class Warper:
    def __init__(self):
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)
         
        # distort src(INPUT) to dst(OUTPUT) 
        # src = np.float32([ # 4개의 원본 좌표 점
        #     [w * 1.6, h * 1.3], # [1024, 624]
        #     [w * (-0.1), h * 1.3], # [-64.0, 624]
        #     [0, h * 0.62], # [0, 297.6]
        #     [w, h * 0.62], # [640, 297.6]
        # ])
        # dst = np.float32([ # 4개의 결과 좌표 점
        #     [w * 0.65, h * 0.98], # [416, 470.4]
        #     [w * 0.35, h * 0.98], # [224, 470.4]
        #     [w * (-0.3), 0], # [-192, 0]
        #     [w * 1.3, 0], # [832, 0]
        # ])
        # src = np.float32([[50, 350], [590, 350], [50, 480], [590, 480]])
        # dst = np.float32([[50,0],[,0],[-192,0],[832,0]])
        src = np.float32([[0, 440], [w,440], [140,300], [500,300]])#좌하, 우하, 좌상, 우상
        
        # 변환된 이미지에서의 목표 좌표(dst)를 정의합니다. 이 예시에서는 간단히 이미지를 평행이동시킵니다.
        # 임의로 이동 거리를 설정해보겠습니다.
        dx, dy = 100, 50  # x축으로 100만큼, y축으로 50만큼 이동
        dst = np.float32([[120, h], [w-120,h], [0,0], [w,0]])

        self.M = cv2.getPerspectiveTransform(src, dst) # self.M : 투시변환 행렬(src to dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src) # self.Minv : 투시변환 행렬(dst to src)

    def warp(self, img): 
        return cv2.warpPerspective(
            img,
            self.M, 
            (img.shape[1], img.shape[0]), # img w, h
            flags=cv2.INTER_LINEAR
        )

    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )