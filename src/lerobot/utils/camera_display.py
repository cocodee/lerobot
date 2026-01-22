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

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class CameraDisplay:
    """
    Lightweight multi-camera display tool supporting various layout options.

    Features:
    - Single window displaying all cameras
    - Customizable layouts (grid, horizontal, vertical)
    - Camera name watermark overlay support
    - Low-latency design, minimal impact on recording performance
    - Automatic handling of cameras with different resolutions
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
        """
        Initialize camera display.

        Args:
            window_name: Window title
            layout: Layout mode - 'grid' (grid), 'horizontal' (side-by-side), 'vertical' (stacked)
            show_names: Whether to show camera names
            name_font_scale: Camera name font scale
            name_color: Camera name text color (BGR)
            name_bg_color: Camera name background color (BGR)
            name_position: Camera name position - 'top-left', 'top-right', 'bottom-left', 'bottom-right'
            max_fps: Maximum display frame rate to avoid excessive CPU usage
            max_display_width: Maximum display width
            max_display_height: Maximum display height
        """
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

        # Store camera data
        self.camera_data: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()

        # Last display time
        self.last_display_time = 0

        # Whether window is created
        self.window_created = False

        # Close flag
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
            # Convert to uint8 if needed
            if image.dtype != np.uint8:
                image = (image * 255).astype(np.uint8)

            # Handle CHW format (common in PyTorch)
            if len(image.shape) == 3 and image.shape[0] == image.shape[1] == 3:
                image = image.transpose(1, 2, 0)

            # Convert RGB to BGR (if in RGB format)
            if len(image.shape) == 3 and image.shape[2] == 3:
                try:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                except cv2.error:
                    pass  # Already in BGR format

        self.camera_data[camera_id] = {
            "image": image,
            "timestamp": time.time(),
        }

    def remove_camera(self, camera_id: str) -> None:
        """Remove a camera."""
        if camera_id in self.camera_data:
            del self.camera_data[camera_id]

    def get_layout_grid(self, num_cameras: int) -> Tuple[int, int]:
        """
        Calculate grid layout rows and columns.

        Args:
            num_cameras: Number of cameras

        Returns:
            (rows, cols): Number of rows and columns
        """
        if num_cameras <= 0:
            return 0, 0

        # Try to get a near-square layout
        cols = int(np.ceil(np.sqrt(num_cameras)))
        rows = int(np.ceil(num_cameras / cols))

        return rows, cols

    def _add_name_overlay(self, image: np.ndarray, camera_id: str) -> np.ndarray:
        """
        Add camera name overlay to image.

        Args:
            image: Input image
            camera_id: Camera identifier

        Returns:
            Processed image with name overlay
        """
        if not self.show_names:
            return image

        # Get text size
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = camera_id
        (text_width, text_height), _ = cv2.getTextSize(text, font, self.name_font_scale, 1)

        # Calculate position
        h, w = image.shape[:2]
        if self.name_position == "top-left":
            x, y = 5, text_height + 5
        elif self.name_position == "top-right":
            x, y = w - text_width - 5, text_height + 5
        elif self.name_position == "bottom-left":
            x, y = 5, h - 5
        elif self.name_position == "bottom-right":
            x, y = w - text_width - 5, h - 5
        else:  # Default to top-left
            x, y = 5, text_height + 5

        # Add background rectangle
        cv2.rectangle(
            image,
            (x - 2, y - text_height - 2),
            (x + text_width + 2, y + 2),
            self.name_bg_color,
            -1,
        )

        # Add text
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
        """
        Combine all camera images according to layout mode.

        Returns:
            Combined image, or None if no cameras
        """
        if not self.camera_data:
            return None

        images = list(self.camera_data.values())
        num_cameras = len(images)

        if num_cameras == 1:
            # Single camera, return directly
            camera_id = list(self.camera_data.keys())[0]
            return self._add_name_overlay(images[0]["image"], camera_id)

        # Get all image shapes
        img_shapes = [img["image"].shape for img in images]

        # Resize all to max dimensions (maintain aspect ratio)
        max_img_height = max(shape[0] for shape in img_shapes)
        max_img_width = max(shape[1] for shape in img_shapes)

        resized_images = []
        camera_names = list(self.camera_data.keys())

        for i, (img_data, name) in enumerate(zip(images, camera_names)):
            h, w = img_data["image"].shape[:2]

            # Resize to maintain uniform size
            if h != max_img_height or w != max_img_width:
                # Maintain aspect ratio
                scale = min(max_img_height / h, max_img_width / w)
                new_w = int(w * scale)
                new_h = int(h * scale)
                img = cv2.resize(img_data["image"], (new_w, new_h), interpolation=cv2.INTER_AREA)
            else:
                img = img_data["image"]

            # Add name watermark
            img = self._add_name_overlay(img, name)
            resized_images.append(img)

        # Combine according to layout
        return self._layout_combine(resized_images)

    def _layout_combine(self, resized_images: List[np.ndarray]) -> np.ndarray:
        """Combine images based on layout configuration."""
        num_cameras = len(resized_images)
        rows, cols = self._get_layout(num_cameras)

        if self.layout == "horizontal":
            # Horizontal layout
            total_width = sum(img.shape[1] for img in resized_images)
            total_height = max(img.shape[0] for img in resized_images)

            combined = np.zeros((total_height, total_width, 3), dtype=np.uint8)
            x_offset = 0
            for img in resized_images:
                h, w = img.shape[:2]
                y_offset = (total_height - h) // 2  # Vertically center
                combined[y_offset : y_offset + h, x_offset : x_offset + w] = img
                x_offset += w

        elif self.layout == "vertical":
            # Vertical layout
            total_width = max(img.shape[1] for img in resized_images)
            total_height = sum(img.shape[0] for img in resized_images)

            combined = np.zeros((total_height, total_width, 3), dtype=np.uint8)
            y_offset = 0
            for img in resized_images:
                h, w = img.shape[:2]
                x_offset = (total_width - w) // 2  # Horizontally center
                combined[y_offset : y_offset + h, x_offset : x_offset + w] = img
                y_offset += h

        else:  # Grid layout (default)
            # Grid layout
            row_heights = []
            col_widths = [0] * cols

            # Calculate max width for each column
            for i, img in enumerate(resized_images):
                col = i % cols
                row_heights.append(img.shape[0])
                col_widths[col] = max(col_widths[col], img.shape[1])

            # Calculate max height for each row
            row_heights = []
            for i in range(rows):
                row_start = i * cols
                row_end = min(row_start + cols, num_cameras)
                row_heights.append(
                    max(
                        resized_images[j].shape[0]
                        for j in range(row_start, row_end)
                        if j < len(resized_images)
                    )
                )

            total_width = sum(col_widths)
            total_height = sum(row_heights)

            combined = np.zeros((total_height, total_width, 3), dtype=np.uint8)

            y_offset = 0
            for i in range(rows):
                x_offset = 0
                row_start = i * cols
                row_end = min(row_start + cols, num_cameras)

                for j in range(row_start, row_end):
                    if j < len(resized_images):
                        img = resized_images[j]
                        h, w = img.shape[:2]
                        combined[y_offset : y_offset + h, x_offset : x_offset + w] = img
                        x_offset += col_widths[j - row_start]

                y_offset += row_heights[i]

        # Limit maximum display size
        combined = self._limit_display_size(combined)
        return combined

    def _limit_display_size(self, combined: np.ndarray) -> np.ndarray:
        """Limit combined image to maximum display dimensions."""
        h, w = combined.shape[:2]

        if w > self.max_display_width:
            scale = self.max_display_width / w
            new_h = int(h * scale)
            combined = cv2.resize(
                combined, (self.max_display_width, new_h), interpolation=cv2.INTER_AREA
            )

        h, w = combined.shape[:2]

        if h > self.max_display_height:
            scale = self.max_display_height / h
            new_w = int(w * scale)
            combined = cv2.resize(
                combined, (new_w, self.max_display_height), interpolation=cv2.INTER_AREA
            )

        return combined

    def _get_layout(self, num_cameras: int) -> Tuple[int, int]:
        """Get layout configuration."""
        if self.layout == "horizontal":
            return 1, num_cameras
        elif self.layout == "vertical":
            return num_cameras, 1
        else:  # Grid
            return self.get_layout_grid(num_cameras)

    def update(self) -> bool:
        """
        Update display by showing combined image in window.

        Returns:
            Whether the display should be closed
        """
        current_time = time.time()

        # Limit display frame rate
        if current_time - self.last_display_time < 1.0 / self.max_fps:
            return self._should_close

        self.last_display_time = current_time

        # Combine and display image
        combined = self.combine_images()
        if combined is not None:
            cv2.imshow(self.window_name, combined)

        # Check if should close
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._should_close = True

        return self._should_close

    def close(self) -> None:
        """Close display window."""
        self._should_close = True
        cv2.destroyAllWindows()

    def should_close(self) -> bool:
        """Check if display should be closed."""
        return self._should_close


def create_camera_display_from_observation(
    observation: Dict[str, Any],
    camera_display: Optional[CameraDisplay] = None,
    camera_keys: Optional[List[str]] = None,
    **kwargs
) -> Tuple[Optional[CameraDisplay], List[str]]:
    """
    Create or update camera display from observation dictionary.

    Args:
        observation: Observation dictionary
        camera_display: Existing camera display object, creates new if None
        camera_keys: Camera key names to display, auto-detects if None
        **kwargs: Arguments passed to CameraDisplay

    Returns:
        (camera_display, found_camera_keys)
    """
    if not CV2_AVAILABLE:
        return camera_display, []

    # Auto-detect camera keys
    if camera_keys is None:
        camera_keys = []
        for key, value in observation.items():
            if isinstance(value, np.ndarray) and "image" in key.lower():
                # Check if it's an image format
                if len(value.shape) >= 2:  # At least HxW
                    camera_keys.append(key)

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
