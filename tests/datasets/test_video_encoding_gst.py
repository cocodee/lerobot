import unittest
import tempfile
import subprocess
import logging
from pathlib import Path
from PIL import Image, ImageDraw

# 假设您的 encode_video_frames 函数存放在名为 video_utils.py 的文件中
# 请根据实际情况修改下面的导入语句
from lerobot.datasets.video_utils import encode_video_frames 

# 配置日志，方便在测试失败时查看 GStreamer 命令
logging.basicConfig(level=logging.INFO)

class TestVideoEncoding(unittest.TestCase):
    
    def setUp(self):
        """在每个测试方法运行前被调用"""
        # 1. 创建一个安全的临时目录
        self.temp_dir = tempfile.TemporaryDirectory()
        self.test_root = Path(self.temp_dir.name)
        
        # 2. 在临时目录中创建存放图片的子目录
        self.imgs_dir = self.test_root / "test_frames"
        self.imgs_dir.mkdir()
        
        # 3. 定义输出视频文件的路径
        self.video_path = self.test_root / "output.mp4"

    def tearDown(self):
        """在每个测试方法运行后被调用，用于清理"""
        self.temp_dir.cleanup()

    def _generate_test_frames(self, num_frames=15, width=128, height=64):
        """一个辅助函数，用于生成测试用的 PNG 图片"""
        for i in range(num_frames):
            # 创建一个黑色背景的图片
            img = Image.new('RGB', (width, height), color='black')
            draw = ImageDraw.Draw(img)
            
            # 在图片上绘制帧编号，方便肉眼验证
            text = f"Frame {i+1}"
            draw.text((10, 10), text, fill='white')
            
            # 保存为符合函数要求的格式
            filename = self.imgs_dir / f"frame_{i:06d}.png"
            img.save(filename)
        return num_frames, width, height

    def test_successful_encoding(self):
        """测试基本功能：成功将图片序列编码为视频"""
        # --- 准备 ---
        num_frames, width, height = self._generate_test_frames()
        fps = 5
        gop_size = 5

        # --- 执行 ---
        encode_video_frames(
            imgs_dir=self.imgs_dir,
            video_path=self.video_path,
            fps=fps,
            g=gop_size,
            overwrite=True # 在测试中总是覆盖
        )

        # --- 验证 ---
        # 1. 基本验证：文件是否存在且不为空
        self.assertTrue(self.video_path.exists(), "视频文件未被创建")
        self.assertGreater(self.video_path.stat().st_size, 0, "视频文件大小为0")

        # 2. 高级验证：使用 ffprobe 检查视频元数据
        try:
            cmd = [
                "ffprobe",
                "-v", "error",
                "-select_streams", "v:0",
                "-show_entries", "stream=width,height,codec_name,r_frame_rate,avg_frame_rate",
                "-of", "default=noprint_wrappers=1:nokey=1",
                str(self.video_path)
            ]
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            output = result.stdout.strip().split('\n')
            
            v_width, v_height, v_codec, v_r_frame_rate, v_avg_frame_rate = output

            self.assertEqual(int(v_width), width, "视频宽度不匹配")
            self.assertEqual(int(v_height), height, "视频高度不匹配")
            self.assertEqual(v_codec, "h264", "视频编码格式不是h264")
            self.assertIn(str(fps), v_r_frame_rate, "视频帧率不匹配") # e.g., "5/1"
            self.assertIn(str(fps), v_avg_frame_rate, "视频平均帧率不匹配")

        except FileNotFoundError:
            self.skipTest("ffprobe 未安装，跳过视频元数据验证。")
        except subprocess.CalledProcessError as e:
            self.fail(f"ffprobe 执行失败: {e.stderr}")

    def test_no_input_images(self):
        """测试当输入目录为空时，是否按预期抛出异常"""
        # imgs_dir 是空的，因为我们没有调用 _generate_test_frames
        with self.assertRaises(FileNotFoundError, msg="当没有图片时应抛出 FileNotFoundError"):
            encode_video_frames(
                imgs_dir=self.imgs_dir,
                video_path=self.video_path,
                fps=10
            )

    def test_overwrite_protection(self):
        """测试在 overwrite=False 的情况下，如果文件已存在是否会抛出异常"""
        # --- 准备 ---
        # 先创建一个空的占位文件
        self.video_path.touch()
        self._generate_test_frames(num_frames=1) # 至少要有一张图片

        # --- 执行与验证 ---
        with self.assertRaises(FileExistsError, msg="当文件存在且 overwrite=False 时应抛出 FileExistsError"):
            encode_video_frames(
                imgs_dir=self.imgs_dir,
                video_path=self.video_path,
                fps=10,
                overwrite=False # 这是默认值，但显式写出更清晰
            )
        
        # 确保文件未被修改
        self.assertEqual(self.video_path.stat().st_size, 0, "受保护的文件不应被修改")

if __name__ == '__main__':
    # 替换 'your_module_name' 为您存放函数的Python文件名（不含.py后缀）
    # 例如，如果文件是 video_utils.py，就写 from video_utils import encode_video_frames
    try:
        from lerobot.datasets.video_utils import encode_video_frames_gst
    except ImportError:
        print("="*80)
        print("错误：请将 'your_module_name' 替换为包含 encode_video_frames 函数的文件名。")
        print("例如：如果您的文件是 'my_script.py'，请修改第12行和第165行的导入语句。")
        print("="*80)
        exit(1)
        
    unittest.main(argv=['first-arg-is-ignored'], exit=False)