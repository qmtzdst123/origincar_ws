import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from volcenginesdkarkruntime import Ark
import base64

class ImageUploadAnalyzer(Node):
    def __init__(self):
        """初始化图像上传分析节点"""
        super().__init__('image_upload_analyzer')
        
        # 声明节点参数
        # 请务必安全地设置您的ARK API Key，避免直接硬编码在代码中。
        self.declare_parameter('ark_api_key', '*********************)
        
        # 创建发布者
        self.result_publisher = self.create_publisher(String, 'image_ai', 10)
        
        # 订阅压缩图像话题
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image',
            self.image_callback,
            10)
        
        # 订阅触发信号话题
        self.sign_sub = self.create_subscription(
            Int32,
            '/sign4return',
            self.sign_callback,
            10)
        
        # 状态变量
        self.trigger_analysis = False
        self.latest_image_data = None
        self.get_logger().info("图像上传分析节点已初始化完成")

    def sign_callback(self, msg):
        """触发信号回调函数"""
        if msg.data == 30:
            self.get_logger().info("收到触发信号(30)，准备分析下一张图像")
            self.trigger_analysis = True

    def image_callback(self, msg):
        """图像数据回调函数"""
        self.latest_image_data = msg.data
        if self.trigger_analysis and self.latest_image_data is not None:
            self.trigger_analysis = False
            self.get_logger().info("开始处理图像...")
            self.process_image(self.latest_image_data)
            self.latest_image_data = None  # 清空已处理图像

    def process_image(self, compressed_data):
        """处理图像数据并直接进行分析"""
        try:
            # 将压缩图像数据转换为numpy数组
            np_arr = np.frombuffer(compressed_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is None:
                self.get_logger().error("图像解码失败")
                return
                
            # 将图像编码为JPEG格式的字节流
            # 可以调整JPEG质量（90表示90%质量）
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90] 
            _, buffer = cv2.imencode('.jpg', img, encode_param)
            
            # 将字节流编码为base64字符串
            base64_image = base64.b64encode(buffer).decode('utf-8')
            
            self.analyze_image_with_ark(base64_image)
            
        except Exception as e:
            self.get_logger().error(f"图像处理过程中发生错误: {str(e)}")

    def analyze_image_with_ark(self, base64_image):
        """使用ARK分析Base64编码的图像"""
        ark_api_key = self.get_parameter('ark_api_key').value
        
        if not ark_api_key or ark_api_key == 'YOUR_ARK_API_KEY':
            self.get_logger().error("ARK API Key 未设置或为默认值，请确保其已正确配置。")
            return

        try:
            # 初始化ARK客户端并调用模型分析
            self.get_logger().info("正在调用模型分析图像数据...")
            client_ark = Ark(api_key=ark_api_key)
            resp = client_ark.chat.completions.create(
                model="doubao-1.5-vision-pro-250328",
                messages=[{
                    "content": [
                        {"text": "描述图中卡片上的内容30字左右", "type": "text"},
                        # 使用数据URI方案传递Base64编码的图像
                        {"image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}, "type": "image_url"}
                    ],
                    "role": "user"
                }]
            )
            
            # 发布分析结果
            result = resp.choices[0].message.content
            msg = String()
            msg.data = f"图像分析结果：{result}"
            self.result_publisher.publish(msg)
            self.get_logger().info(f"分析完成: {result}")
            
        except Exception as e:
            self.get_logger().error(f"ARK分析过程中发生错误: {str(e)}")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    try:
        node = ImageUploadAnalyzer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("节点已关闭")

if __name__ == '__main__':
    main()