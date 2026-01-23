# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Lightweight multi-camera display utility that shows multiple camera feeds in a single window.
Uses OpenCV for low-latency real-time display.
"""

import time
from collections import OrderedDict
from typing import Any, Dict, List, Optional, Tuple
import logging
import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

logger = logging.getLogger(__name__)

class CameraDisplay:
    """
    Lightweight multi-camera display tool supporting various layout options.
    """

    def __init__(
        self,
        window_name: str = "Camera Feed",
        layout: str = "grid",
        show_names: bool = True,
        name_font_scale: float = 0.7,
        name_color: Tuple[int, int, int] = (255, 255, 255),
        name_bg_color: Tuple[int, int, int] = (0, 0, 0),
        name_position: str = "top-left",
        max_fps: int = 30,
        max_display_width: int = 1920,
        max_display_height: int = 1080,
    ):
        if not CV2_AVAILABLE:
            raise ImportError("OpenCV is required for camera display. Install with: pip install opencv-python")

        self.window_name = window_name
        self.layout = layout
        self.show_names = show_names
        self.name_font_scale = name_font_scale
        self.name_color = name_color
        self.name_bg_color = name_bg_color
        self.name_position = name_position
        self.max_fps = max_fps
        self.max_display_width = max_display_width
        self.max_display_height = max_display_height

        self.camera_data: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()
        self.last_display_time = 0
        self.window_created = False
        self._should_close = False

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    def add_camera(self, camera_id: str, image: Any) -> None:
        """
        Add or update camera image.
        Args:
            camera_id: Camera identifier
            image: Image data (numpy array, RGB or BGR format)
        """
        import numpy as np

        # Convert to BGR format if needed
        if isinstance(image, np.ndarray) and len(image.shape) >= 2:
            # Handle float images (0.0-1.0) -> uint8
            if image.dtype != np.uint8:
                if image.max() <= 1.0:
                    image = (image * 255).astype(np.uint8)
                else:
                    image = image.astype(np.uint8)

            # [修正] 改进 CHW (Channel-First) 格式检测逻辑
            # 原代码逻辑有误，这里检查是否第一维是 3 (RGB) 或 1 (Gray)，且后续维度更大
            if len(image.shape) == 3:
                c, h, w = image.shape
                # 如果 C 是 1 或 3，且 H 和 W 都大于 C，通常意味着是 (C, H, W) 格式
                if c in [1, 3] and h > c and w > c:
                    image = image.transpose(1, 2, 0)

            # Convert RGB to BGR (if in RGB format)
            # OpenCV assumes BGR, most envs provide RGB
            if len(image.shape) == 3 and image.shape[2] == 3:
                try:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                except cv2.error:
                    pass 

        self.camera_data[camera_id] = {
            "image": image,
            "timestamp": time.time(),
        }

    def remove_camera(self, camera_id: str) -> None:
        if camera_id in self.camera_data:
            del self.camera_data[camera_id]

    def get_layout_grid(self, num_cameras: int) -> Tuple[int, int]:
        if num_cameras <= 0:
            return 0, 0
        cols = int(np.ceil(np.sqrt(num_cameras)))
        rows = int(np.ceil(num_cameras / cols))
        return rows, cols

    def _add_name_overlay(self, image: np.ndarray, camera_id: str) -> np.ndarray:
        if not self.show_names:
            return image

        font = cv2.FONT_HERSHEY_SIMPLEX
        text = camera_id
        (text_width, text_height), _ = cv2.getTextSize(text, font, self.name_font_scale, 1)

        h, w = image.shape[:2]
        if self.name_position == "top-left":
            x, y = 5, text_height + 5
        elif self.name_position == "top-right":
            x, y = w - text_width - 5, text_height + 5
        elif self.name_position == "bottom-left":
            x, y = 5, h - 5
        elif self.name_position == "bottom-right":
            x, y = w - text_width - 5, h - 5
        else:
            x, y = 5, text_height + 5

        cv2.rectangle(
            image,
            (x - 2, y - text_height - 2),
            (x + text_width + 2, y + 2),
            self.name_bg_color,
            -1,
        )

        cv2.putText(
            image,
            text,
            (x, y),
            font,
            self.name_font_scale,
            self.name_color,
            1,
            cv2.LINE_AA,
        )
        return image

    def combine_images(self) -> Optional[np.ndarray]:
        if not self.camera_data:
            return None

        images = list(self.camera_data.values())
        num_cameras = len(images)

        if num_cameras == 1:
            camera_id = list(self.camera_data.keys())[0]
            return self._add_name_overlay(images[0]["image"], camera_id)

        img_shapes = [img["image"].shape for img in images]
        max_img_height = max(shape[0] for shape in img_shapes)
        max_img_width = max(shape[1] for shape in img_shapes)

        resized_images = []
        camera_names = list(self.camera_data.keys())

        for i, (img_data, name) in enumerate(zip(images, camera_names)):
            h, w = img_data["image"].shape[:2]
            if h != max_img_height or w != max_img_width:
                scale = min(max_img_height / h, max_img_width / w)
                new_w = int(w * scale)
                new_h = int(h * scale)
                img = cv2.resize(img_data["image"], (new_w, new_h), interpolation=cv2.INTER_AREA)
            else:
                img = img_data["image"]
            img = self._add_name_overlay(img, name)
            resized_images.append(img)

        return self._layout_combine(resized_images)

    def _layout_combine(self, resized_images: List[np.ndarray]) -> np.ndarray:
        num_cameras = len(resized_images)
        rows, cols = self._get_layout(num_cameras)

        if self.layout == "horizontal":
            total_width = sum(img.shape[1] for img in resized_images)
            total_height = max(img.shape[0] for img in resized_images)
            combined = np.zeros((total_height, total_width, 3), dtype=np.uint8)
            x_offset = 0
            for img in resized_images:
                h, w = img.shape[:2]
                y_offset = (total_height - h) // 2
                combined[y_offset : y_offset + h, x_offset : x_offset + w] = img
                x_offset += w

        elif self.layout == "vertical":
            total_width = max(img.shape[1] for img in resized_images)
            total_height = sum(img.shape[0] for img in resized_images)
            combined = np.zeros((total_height, total_width, 3), dtype=np.uint8)
            y_offset = 0
            for img in resized_images:
                h, w = img.shape[:2]
                x_offset = (total_width - w) // 2
                combined[y_offset : y_offset + h, x_offset : x_offset + w] = img
                y_offset += h

        else:  # Grid
            # 简化 Grid 计算逻辑以避免原代码中潜在的索引越界或对齐问题
            row_heights = []
            col_widths = [0] * cols
            
            # 计算每列最大宽度
            for i, img in enumerate(resized_images):
                col = i % cols
                col_widths[col] = max(col_widths[col], img.shape[1])
            
            # 计算每行最大高度
            for r in range(rows):
                max_h = 0
                for c in range(cols):
                    idx = r * cols + c
                    if idx < num_cameras:
                        max_h = max(max_h, resized_images[idx].shape[0])
                row_heights.append(max_h)

            total_width = sum(col_widths)
            total_height = sum(row_heights)
            combined = np.zeros((total_height, total_width, 3), dtype=np.uint8)

            y_offset = 0
            for r in range(rows):
                x_offset = 0
                for c in range(cols):
                    idx = r * cols + c
                    if idx < num_cameras:
                        img = resized_images[idx]
                        h, w = img.shape[:2]
                        # 居中放置
                        y_pos = y_offset + (row_heights[r] - h) // 2
                        x_pos = x_offset + (col_widths[c] - w) // 2
                        combined[y_pos : y_pos + h, x_pos : x_pos + w] = img
                    x_offset += col_widths[c]
                y_offset += row_heights[r]

        return self._limit_display_size(combined)

    def _limit_display_size(self, combined: np.ndarray) -> np.ndarray:
        h, w = combined.shape[:2]
        if w > self.max_display_width:
            scale = self.max_display_width / w
            new_h = int(h * scale)
            combined = cv2.resize(combined, (self.max_display_width, new_h), interpolation=cv2.INTER_AREA)
        h, w = combined.shape[:2]
        if h > self.max_display_height:
            scale = self.max_display_height / h
            new_w = int(w * scale)
            combined = cv2.resize(combined, (new_w, self.max_display_height), interpolation=cv2.INTER_AREA)
        return combined

    def _get_layout(self, num_cameras: int) -> Tuple[int, int]:
        if self.layout == "horizontal":
            return 1, num_cameras
        elif self.layout == "vertical":
            return num_cameras, 1
        else:
            return self.get_layout_grid(num_cameras)

    def update(self) -> bool:
        current_time = time.time()
        if current_time - self.last_display_time < 1.0 / self.max_fps:
            return self._should_close
        self.last_display_time = current_time
        combined = self.combine_images()
        if combined is not None:
            cv2.imshow(self.window_name, combined)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._should_close = True
        return self._should_close

    def close(self) -> None:
        self._should_close = True
        cv2.destroyAllWindows()

    def should_close(self) -> bool:
        return self._should_close


def create_camera_display_from_observation(
    observation: Dict[str, Any],
    camera_display: Optional[CameraDisplay] = None,
    camera_keys: Optional[List[str]] = None,
    **kwargs
) -> Tuple[Optional[CameraDisplay], List[str]]:
    """
    Create or update camera display from observation dictionary.
    """
    if not CV2_AVAILABLE:
        return camera_display, []

    # Auto-detect camera keys if not provided
    if camera_keys is None:
        camera_keys = []
        for key, value in observation.items():
            # -------------------------------------------------------------
            # [修正重点]
            # 移除 "image" in key 的字符串匹配
            # 改为基于维度判断：
            # 1. 必须是 numpy 数组
            # 2. 必须是 3 维 (图像)
            # 3. 排除掉可能是 3D 的非图像状态（通过检查最小维度，Channel通常很小）
            # -------------------------------------------------------------
            if isinstance(value, np.ndarray) and value.ndim == 3:
                # 检查最小维度是否像 Channel (通常 1, 3, 4)
                # 这可以有效区分 3D 图像 (H, W, 3) 和 2D 状态的 Batch (Batch, Dim) 或者其他 Tensor
                if min(value.shape) <= 4:
                     logger.info(f"Detected camera: {key}, shape: {value.shape}")
                     camera_keys.append(key)
            # -------------------------------------------------------------
            
    if camera_keys:
        logger.info(f"Found {len(camera_keys)} cameras: {camera_keys}")
    
    # Return if no cameras
    if not camera_keys:
        return camera_display, camera_keys

    # Create or get camera display
    if camera_display is None:
        camera_display = CameraDisplay(**kwargs)

    # Update camera images
    for key in camera_keys:
        if key in observation:
            camera_display.add_camera(key, observation[key])

    return camera_display, camera_keys