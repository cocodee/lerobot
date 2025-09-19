from pathlib import Path
from lerobot.datasets.video_utils import encode_video_frames

# 假设你的PNG图片在 './my_frames' 文件夹
image_directory = Path("./outputs/cam_capture/2025-09-18/10-36-45/")
FRAME_RATE = 30

# --- 调用硬件加速编码 (H.264) ---
# 注意：我们没有传递新的参数，函数签名保持不变
# 函数内部会自动检测到 'h264_v4l2m2m' 并使用硬件编码逻辑
print("Starting hardware-accelerated encoding...")
encode_video_frames(
    imgs_dir=image_directory,
    video_path="./output_hw.mp4",
    fps=FRAME_RATE,
    vcodec="h264_v4l2m2m",  # <-- 关键在这里！
    crf=23,                # <-- 这个参数会被函数内部的逻辑忽略
    overwrite=True
)

# --- 调用硬件加速编码 (HEVC/H.265) ---
print("\nStarting HEVC hardware-accelerated encoding...")
encode_video_frames(
    imgs_dir=image_directory,
    video_path="./output_hevc_hw.mp4",
    fps=FRAME_RATE,
    vcodec="hevc_v4l2m2m", # <-- 使用HEVC硬件编码器
    overwrite=True
)


# --- 调用软件编码 (作为对比) ---
# 函数会使用默认的软件编码逻辑
print("\nStarting software encoding for comparison...")
encode_video_frames(
    imgs_dir=image_directory,
    video_path="./output_cpu.mp4",
    fps=FRAME_RATE,
    vcodec="h264",         # 使用通用的h264（通常会链接到libx264）
    crf=23,                # <-- 这个参数在这里会生效
    overwrite=True
)