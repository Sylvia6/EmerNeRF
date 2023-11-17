from PIL import Image
import cv2
import numpy as np


def get_image_size(path):
    img = Image.open(path)
    width, height = img.size
    return width, height


class ImageConcater(object):
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.images = []

    def clear(self):
        self.images = []

    def add_image(self, image, row, col, rows=1, cols=1, keep_ratio=True, text=None):
        self.images.append(
            [image, row, col, rows, cols, keep_ratio, text]
        )

    def render(self):
        max_cols = 0
        max_rows = 0

        for _, row, col, rows, cols, _, _ in self.images:
            max_rows = max(max_rows, row+rows)
            max_cols = max(max_cols, col+cols)

        result = np.zeros((max_rows*self.grid_size[0],
                           max_cols*self.grid_size[1], 3), dtype='uint8')

        for image, row, col, rows, cols, keep_ratio, text in self.images:
            max_h = rows * self.grid_size[0]
            max_w = cols * self.grid_size[1]
            im_h, im_w = image.shape[:2]
            if keep_ratio:
                if im_h / float(im_w) > max_h / float(max_w):
                    dst_h = max_h
                    dst_w = int(im_w * max_h / float(im_h))
                else:
                    dst_w = max_w
                    dst_h = int(im_h * max_w / float(im_w))
            else:
                dst_w, dst_h = max_w, max_h
            dst_image = cv2.resize(image, (dst_w, dst_h))
            if text is not None:
                cv2.putText(dst_image, text, (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8 * max_w / 480, (0, 0, 255), 1)

            shift_w = (max_w - dst_w) // 2
            shift_h = (max_h - dst_h) // 2
            start_h = row * self.grid_size[0]
            start_w = col * self.grid_size[1]

            result[start_h+shift_h:start_h+shift_h+dst_h,
                   start_w+shift_w:start_w+shift_w+dst_w] = dst_image
        return result


class ImageGrid(ImageConcater):
    def __init__(self, grid_size, col):
        super(ImageGrid, self).__init__(grid_size)
        self.col = col

    def add(self, image, text=None):
        image_num = len(self.images)
        row = image_num // self.col
        col = image_num % self.col
        self.add_image(image, row, col, text=text)
