"""仿真 YOLO 替代：返回 ground-truth 检测框。"""

from __future__ import annotations

import numpy as np


class SimDetectionStub:
    """兼容 ultralytics YOLO.predict 接口。"""

    def __init__(self, env, default_class: str = "bottle") -> None:
        self.env = env
        self.default_class = default_class
        self.names = {39: "bottle", 41: "cup", 47: "apple"}

    def predict(self, source, conf=0.5, device=None, verbose=False):
        from types import SimpleNamespace

        p_cam = self.env.object_cam_position(self.default_class)
        uv = self.env.project_cam_to_pixel(p_cam)
        if uv is None:
            return [SimpleNamespace(boxes=None, names=self.names)]
        u, v = uv
        bw, bh = 45, 55
        box = SimpleNamespace(
            xyxy=np.array([[u - bw, v - bh, u + bw, v + bh]], dtype=np.float32),
            cls=np.array([float(39 if self.default_class == "bottle" else 41)], dtype=np.float32),
            conf=np.array([0.93], dtype=np.float32),
        )

        class _Boxes:
            def __iter__(self):
                return iter([box])

            def __len__(self):
                return 1

        result = SimpleNamespace(boxes=_Boxes(), names=self.names)
        return [result]
