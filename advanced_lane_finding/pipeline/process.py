import cv2
import numpy as np

from pipeline.gradient import Gradient

class Process(object):

    def __init__(self, gradient, camera, perspective, curvature):
        self.camera = camera
        self.gradient = gradient
        self.perspective = perspective
        self.curvature = curvature
    
    def _perspective(self, img):
        """transform the perspective using the source and destination.
        Args:
            src: A `Rectangle`
            dst: A `Rectangle`
        Returns:
            An image
        """
        M = cv2.getPerspectiveTransform(self.perspective.src.to_array(),
                                        self.perspective.dst.to_array())
        Minv = cv2.getPerspectiveTransform(self.perspective.dst.to_array(),
                                           self.perspective.src.to_array())
        img_size = (img.shape[1], img.shape[0])
        return cv2.warpPerspective(img, M, img_size), Minv

    def _add_text(self, img, left_curve, right_curve, offset):
        curve_text = 'Curvature : {0:.3f}m - {1:.3f}m'.format(left_curve,
                                                              right_curve)
        offset_text = 'Offset: {:.3f}'.format(offset)
        cv2.putText(img, curve_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
        cv2.putText(img, offset_text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)

        return img


    def process(self, img):
        undistorted = self.camera.undistort(img)

        img = self.gradient.process(undistorted)
        warped, matrix_inv = self._perspective(img)

        curv = self.curvature
        curv.process(warped)

        left_curve, right_curve = curv.curvature()
        offset = curv.get_offset(img)

        undistorted = self._add_text(undistorted, left_curve, right_curve, offset)
        undistorted = curv.draw_line(undistorted, warped, matrix_inv)

        return undistorted
