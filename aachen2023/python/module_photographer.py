# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys

class module_photographer():
    def __init__(self):
        self.g_name = "g_ref_img"
        self.r_name = "r_ref_img"
        self.WIDTH = 640
        self.HEIGHT = 480
        self.range_max = 10.0
        self.range_min = 0.105
        #self.range_max = 100.0
        #self.range_min = 0.0

        self.bg = 12

        # ストリーミング初期化
        config = rs.config()
        config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)

        # ストリーミング開始
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(config)
        # Alignオブジェクト生成
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.pipeline.stop()

    def r_run(self):
        self.pipeline.start()
        time.sleep(3)
        threshold = (self.WIDTH * self.HEIGHT * 3) * 0.95

        depth_scale = self.depth_sensor.get_depth_scale()
        clipping_distance_in_meters = 0.6 # m以内を検出
        clipping_distance = clipping_distance_in_meters / depth_scale

        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # clipping_distance_in_metersm以内を画像化
            white_color = 255 # 背景色
            depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), white_color, color_image)
            # 背景色となっているピクセル数をカウント
            white_pic = np.sum(bg_removed == 255)
            # 背景色が一定値以下になった時に、「検出」を表示する
            #if(threshold > white_pic):
                #print("検出 {}".format(white_pic))
            #else:
                #print("{}".format(white_pic))

            bg_removed2 = color_image.copy()
            bg_removed_top_x = int(bg_removed2.shape[1])
            bg_removed_top_y = np.where(bg_removed != 255)[0][0]
            cv2.rectangle(bg_removed2, (0,0), (bg_removed_top_x, bg_removed_top_y), (255, 255, 255), thickness=-1)

            #images = np.hstack((bg_removed2, color_image))
            cv2.imwrite('{}.jpg'.format(self.r_name), bg_removed2)


            # ストリーミング停止
            self.pipeline.stop()
            cv2.destroyAllWindows()

        except KeyboardInterrupt:
            # ストリーミング停止
            self.pipeline.stop()
            cv2.destroyAllWindows()

    def g_run(self):
        self.pipeline.start()
        time.sleep(3)
        depth_scale = self.depth_sensor.get_depth_scale()

        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            depth = depth_image.astype(np.float64) * depth_scale
            depth = np.where((depth > self.range_min) & (depth < self.range_max), depth - self.range_min, 0)
            depth = depth * ( 255 / (self.range_max - self.range_min) )
            depth = depth.astype(np.uint8)

            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            depth_image3 = np.where(((depth < self.bg) & (depth > 0)), 255, 0)
            depth_image3 = depth_image3.astype(np.uint8)

            color_image3 = color_image * ((depth_image3/255).astype(np.uint8))

            images = color_image3

            cv2.imwrite('{}.jpg'.format(self.g_name), images)

            # ストリーミング停止
            self.pipeline.stop()
            cv2.destroyAllWindows()

        except KeyboardInterrupt:
            # ストリーミング停止
            self.pipeline.stop()
            cv2.destroyAllWindows()


def main():
    inst = module_photographer()
    inst.r_run()
    inst.g_run()

if __name__ == "__main__":
    main()
