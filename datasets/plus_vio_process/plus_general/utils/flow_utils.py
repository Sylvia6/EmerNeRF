import cv2
import numpy as np


def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    if len(img.shape) == 2:
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    else:
        vis = img.copy()
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis


def resize_flow(flow, dst_width, dst_height):
    sx = dst_width / float(flow.shape[1])
    sy = dst_height / float(flow.shape[0])
    flow = cv2.resize(flow, (dst_width, dst_height))
    flow[..., 0] *= sx
    flow[..., 1] *= sy
    return flow


def warp_flow(img, flow):
    h, w = flow.shape[:2]
    flow = flow
    flow[:,:,0] += np.arange(w)
    flow[:,:,1] += np.arange(h)[:, np.newaxis]
    res = cv2.remap(img, flow, None, cv2.INTER_LINEAR)
    return res