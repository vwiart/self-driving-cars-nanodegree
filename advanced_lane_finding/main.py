import os

import glob
import pickle

import camera
import image
from pipeline.curvature import Curvature
from pipeline.gradient import Gradient, Sobel
from pipeline.perspective import Perspective, Point, Rectangle
from pipeline.process import Process
import video



def main():
    cam = camera.Camera()

    images = glob.glob('camera_cal/calibration*.jpg')
    cam.calibrate(images)
    
    l_sobel = Sobel(threshold_min=225, threshold_max=255)
    b_sobel = Sobel(threshold_min=190, threshold_max=255)
    abs_sobel = Sobel(kernel=9, threshold_min=20, threshold_max=255)

    gradient = Gradient(abs_sobel, l_sobel, b_sobel)

    src = Rectangle(Point(562, 474),
                    Point(724, 474),
                    Point(1005, 663),
                    Point(298, 663))
    dst = Rectangle(Point(320, 0),
                    Point(1005, 0),
                    Point(1005, 663),
                    Point(320, 663))

    perspective_transform = Perspective(src, dst)

    curvature = Curvature(number=9, margin=50, min_pix = 100)

    process_image = Process(gradient=gradient,
                            camera=cam,
                            perspective=perspective_transform,
                            curvature=curvature)
    video.dump('project_video.mp4', 'output_images/', process_image.process)


if __name__ == '__main__':
    main()
