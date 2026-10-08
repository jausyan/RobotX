"""YOLOv8-style ONNX detector - Python port of VisionVideoNode::infer().

Backends (same two as the C++ vision_video package):
  * OpenVINO runtime  (use_openvino=True, device e.g. "NPU" / "CPU" / "GPU")
  * OpenCV DNN        (use_openvino=False)

Expected model output: (1, 4+nc, N) or (1, N, 4+nc) with rows cx, cy, w, h,
class scores (no objectness) - i.e. a default Ultralytics ONNX export.
For NPU the model must have a static input shape (the default export does).
"""
import ast

import cv2
import numpy as np


def read_onnx_names(path):
    """Class names embedded by the Ultralytics exporter, or [] if unavailable."""
    try:
        import onnx
        model = onnx.load(path, load_external_data=False)
        for prop in model.metadata_props:
            if prop.key == 'names':
                names = ast.literal_eval(prop.value)
                return [names[i] for i in sorted(names)]
    except Exception:
        pass
    return []


class Detector:
    def __init__(self, path, in_w, in_h, conf, nms, use_openvino=False,
                 ov_device='CPU', letterbox=True):
        self.in_w, self.in_h = int(in_w), int(in_h)
        self.conf, self.nms = float(conf), float(nms)
        self.letterbox = letterbox
        self.num_classes = -1
        self.req = self.net = None

        if use_openvino:
            import openvino as ov
            core = ov.Core()
            self.compiled = core.compile_model(core.read_model(path), ov_device)
            self.req = self.compiled.create_infer_request()
            self.backend = f'OpenVINO[{ov_device}]'
        else:
            self.net = cv2.dnn.readNetFromONNX(path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.backend = 'OpenCV-DNN'

        self.infer(np.zeros((self.in_h, self.in_w, 3), np.uint8))   # warm-up, sets num_classes

    # ------------------------------------------------------------------
    def _preprocess(self, frame):
        """-> (image, (x0, y0, sx, sy)) with frame_xy = (model_xy - (x0, y0)) * (sx, sy)."""
        h, w = frame.shape[:2]
        if not self.letterbox:                       # plain stretch, like blobFromImage
            return frame, (0.0, 0.0, w / self.in_w, h / self.in_h)
        r = min(self.in_w / w, self.in_h / h)
        nw, nh = int(round(w * r)), int(round(h * r))
        x0, y0 = (self.in_w - nw) // 2, (self.in_h - nh) // 2
        canvas = np.full((self.in_h, self.in_w, 3), 114, np.uint8)
        canvas[y0:y0 + nh, x0:x0 + nw] = cv2.resize(frame, (nw, nh))
        return canvas, (float(x0), float(y0), 1.0 / r, 1.0 / r)

    def _forward(self, blob):
        if self.req is not None:
            self.req.infer({0: blob})
            return np.array(self.req.get_output_tensor(0).data)
        self.net.setInput(blob)
        return self.net.forward()

    def infer(self, frame):
        """-> [(class_id, confidence, u_center, v_center, w, h), ...] in frame pixels."""
        img, tf = self._preprocess(frame)
        blob = cv2.dnn.blobFromImage(img, 1.0 / 255.0, (self.in_w, self.in_h),
                                     swapRB=True, crop=False)
        return self._decode(self._forward(blob), tf, frame.shape)

    def _decode(self, out, tf, shape):
        out = np.asarray(out)
        out = out.reshape(out.shape[-2], out.shape[-1])
        if out.shape[0] < out.shape[1]:              # channels-first -> (N, 4+nc)
            out = out.T
        self.num_classes = out.shape[1] - 4
        if self.num_classes < 1:
            return []

        scores = out[:, 4:]
        cls = scores.argmax(axis=1)
        conf = scores[np.arange(len(cls)), cls]
        keep = conf >= self.conf
        if not keep.any():
            return []
        b, cls, conf = out[keep, :4], cls[keep], conf[keep]

        x0, y0, sx, sy = tf
        cx, cy = (b[:, 0] - x0) * sx, (b[:, 1] - y0) * sy
        w, h = b[:, 2] * sx, b[:, 3] * sy
        H, W = shape[:2]
        x1, y1 = np.clip(cx - w / 2, 0, W), np.clip(cy - h / 2, 0, H)
        x2, y2 = np.clip(cx + w / 2, 0, W), np.clip(cy + h / 2, 0, H)
        bw, bh = x2 - x1, y2 - y1
        ok = (bw > 0) & (bh > 0)
        if not ok.any():
            return []
        boxes = np.stack([x1, y1, bw, bh], axis=1)[ok]
        cls, conf = cls[ok], conf[ok]

        idx = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), self.conf, self.nms)
        idx = np.array(idx).flatten().astype(int)
        return [(int(cls[i]), float(conf[i]),
                 float(boxes[i, 0] + boxes[i, 2] / 2), float(boxes[i, 1] + boxes[i, 3] / 2),
                 float(boxes[i, 2]), float(boxes[i, 3])) for i in idx]
