import cv2
import numpy as np

class Curvature(object):

    def __init__(self, number, margin, min_pix):
        self.number = number
        self.margin = margin
        self.min_pix = min_pix
        self.left = None
        self.right = None
        self.first_frame = True
    
    def _histogram(self, img):
        return np.sum(img[img.shape[0]//2:, :], axis=0)
    

    def process_first_frame(self, img):
        histo = self._histogram(img)

        output_img = np.dstack((img, img, img)) * 255

        mid = np.int(histo.shape[0]/2)
        left = np.argmax(histo[:mid])
        right = np.argmax(histo[mid:]) + mid

        height = np.int(img.shape[0] / self.number)

        nonzero = img.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        left_current = left
        right_current = right

        left_lane_indices = []
        right_lane_indices = []

        for window in range(self.number):
            y_low = img.shape[0] - (window+1) * height
            y_high = img.shape[0] - window * height

            bottom_left = left_current - self.margin
            top_left = left_current + self.margin
            bottom_right = right_current - self.margin
            top_right = right_current + self.margin

            cv2.rectangle(output_img, (bottom_left, y_low), (top_left, y_high), (0, 255, 0), 2)
            cv2.rectangle(output_img, (bottom_right, y_low), (top_right, y_high), (0, 255, 0), 2)

            good = lambda top, bottom: ((nonzero_y >= y_low) & (nonzero_y < y_high) & (nonzero_x >= bottom)  & (nonzero_x < top)).nonzero()[0]
            good_left_inds = good(top_left, bottom_left)
            good_right_inds = good(top_right, bottom_right)

            left_lane_indices.append(good_left_inds)
            right_lane_indices.append(good_right_inds)

            if len(good_left_inds) > self.min_pix:
                left_current = np.int(np.mean(nonzero_x[good_left_inds]))
            if len(good_right_inds) > self.min_pix:        
                right_current = np.int(np.mean(nonzero_x[good_right_inds]))

        left_lane_indices = np.concatenate(left_lane_indices)
        right_lane_indices = np.concatenate(right_lane_indices)

        return left_lane_indices, right_lane_indices

    def process(self, img):
        nonzero = img.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])
        left_lane_indices, right_lane_indices = self.process_first_frame(img)
        if not self.first_frame:
            activate = lambda lane: (nonzero_x > polynomial(x=nonzero_y, coef=lane, b=-self.margin)) & \
                                    (nonzero_x < polynomial(x=nonzero_y, coef=lane, b= self.margin))
            left_lane_indices = activate(self.left)
            right_lane_indices = activate(self.right)

        # Extract left and right line pixel positions
        left_x = nonzero_x[left_lane_indices]
        left_y = nonzero_y[left_lane_indices] 
        right_x = nonzero_x[right_lane_indices]
        right_y = nonzero_y[right_lane_indices] 

        if len(left_x) == 0 | len(left_y) == 0:
            return  # skip frame
        # Fit a second order polynomial to each
        left = np.polyfit(left_y, left_x, 2)
        right = np.polyfit(right_y, right_x, 2)

        self.first_frame = False

        lcurve, rcurve = curvature(left, right)

        ratio = abs((lcurve-rcurve) / (lcurve + rcurve))
        if ratio > 0.5:
            return  # skip frame

        self.left = left
        self.right = right

    def curvature(self):
        return curvature(self.left, self.right)
        

    def get_offset(self, img):
        xm_ppx = 3.7 / 700.0
        img_center = img.shape[0] / 2.0

        l = polynomial(img.shape[0], self.left)
        r = polynomial(img.shape[0], self.left)
        lane_center = (l + r) / 2.0

        return (img_center - lane_center) * xm_ppx

    def draw_line(self, img, warped, Minv):
        ploty = np.linspace(0, 719, num=720)
        left_fitx = polynomial(ploty, self.left)
        right_fitx = polynomial(ploty, self.right)

        zeros = np.zeros_like(warped).astype(np.uint8)
        mask = np.dstack((zeros, zeros, zeros))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        cv2.fillPoly(mask, np.int_([pts]), (0,200, 0))
        unwarped = cv2.warpPerspective(mask, Minv, (img.shape[1], img.shape[0]))

        img = cv2.addWeighted(img, 1, unwarped, 0.3, 0)
        return img

def polynomial(x, coef, b=0):
    """polynomial = aX² + bX + c + b"""
    return coef[0] * x ** 2 + coef[1] * x + coef[2] + b

def curvature(left, right):
    ploty = np.linspace(0, 719, num=720)
    ym_ppx = 3.0 / 72.0
    xm_ppx = 3.7 / 700.0

    leftx, rightx = polynomial(ploty, left), polynomial(ploty, right)

    fit_cr = lambda x: np.polyfit(ploty  * ym_ppx, x * xm_ppx, 2)
    left_fit_cr, right_fit_cr = fit_cr(leftx), fit_cr(rightx)

    compute = lambda x: ((1 + (2 * x[0] * np.max(ploty) * ym_ppx + x[1]) ** 2) ** 1.5) / np.absolute(2 * x[0])
    left_curverad = compute(left_fit_cr)
    right_curverad = compute(right_fit_cr)

    return left_curverad, right_curverad