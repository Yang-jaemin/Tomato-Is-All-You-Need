import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import MoveToPosition
from my_interfaces.srv import GetRgbDepth
from cv_bridge import CvBridge
from inference_sdk import InferenceHTTPClient
import cv2
import numpy as np


class SegAnd3DNode(Node):

    def __init__(self):
        super().__init__('segmentation_node')
        self.cli = self.create_client(GetRgbDepth, 'get_rgb_depth')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')
        self.req = GetRgbDepth.Request()
        self.bridge = CvBridge()
        self.image_number = 0
        self.enable = True

        self.action_client = ActionClient(self, MoveToPosition, 'move_to_position')

        # 카메라 인트린식 파라미터 (예시 값)
        self.fx = 525.0  # 초점 거리 x
        self.fy = 525.0  # 초점 거리 y
        self.cx = 319.5  # 카메라 중심 x
        self.cy = 239.5  # 카메라 중심 y

        # 카메라의 extrinsic 매트릭스 (예시 값)
        self.extrinsic_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def segmentation3d(self, rgb_image, depth_image):
        # RGB 이미지를 OpenCV 이미지로 변환
        cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_image, "bgr8")
        self.image_number += 1
        
        # 이미지를 JPG 파일로 저장
        image_path = '../input_images/'+f'{self.image_number}'+'.jpg'  # 순서마다 i 지정
        image = Image.open(image_path)
        
        # Depth 이미지를 OpenCV 이미지로 변환
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image, "passthrough")
        
        # 각 instance의 중심 좌표에서 depth 값을 추출하고 가장 가까운 instance를 찾음
        closest_instance = None
        min_depth = float('inf')

        cv2.imwrite(image_path, cv_rgb_image)
        
        # 저장 되었다는 log
        # self.get_logger().info(f'RGB image saved at {image_path}')
        
        # model inference using API
        
        CLIENT = InferenceHTTPClient(
        api_url="https://detect.roboflow.com",
        api_key="TsYL4fFK3xDuSKuMZk3a"
        )
     
        result = CLIENT.infer(image_path, model_id="tomato-instance-segmentation-t2uhb/2")
        
        segmentations = result['predictions']

        # Plot the image
        fig, ax = plt.subplots(1, figsize=(12, 12))
        ax.imshow(image)

        # Add segmentation areas and labels 
        for segmentation in segmentations:
            # matching rgb & depth
            if segmentation["class"] == "ripened":
                x_center = int(segmentation['x'])
                y_center = int(segmentation['y'])
                depth_value = cv_depth_image[y_center, x_center]
            
                if depth_value < min_depth:
                    min_depth = depth_value
                    closest_instance = segmentation
                
            points = [(point['x'], point['y']) for point in segmentation["points"]]
            polygon = Polygon(points, closed=True, edgecolor='r' if segmentation["class"] == "ripened" else 'g', facecolor='none', linewidth=2)
            
            # Add the polygon to the Axes
            ax.add_patch(polygon)
            ax.text(points[0][0], points[0][1] - 10, f'{segmentation["class"]}: {segmentation["confidence"]:.2f}',
                    bbox=dict(facecolor='yellow', alpha=0.5), fontsize=12, color='black')
            
            # Calculate the center point
            x_center = int(segmentation['x'])
            y_center = int(segmentation['y'])
            
            # Add a dot at the center point
            ax.add_patch(Circle((x_center, y_center), radius=5, color='blue'))
            
        # Save the annotated image
        image_path = '../output_images/'+f'{self.image_number}'+'.jpg'
        plt.axis('off')
        plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
        
        
        # 가장 가까운 instance의 월드 좌표 계산
        if closest_instance:
            x_center = int(closest_instance['x'])
            y_center = int(closest_instance['y'])
            depth_value = cv_depth_image[y_center, x_center]
            
            camera_coords = self.pixel_to_camera_coords(x_center, y_center, depth_value)
            world_coords = self.camera_to_world_coords(camera_coords)
            
            self.get_logger().info(f'Closest instance: {closest_instance}, Depth: {min_depth}, World Coordinates: {world_coords}')
            self.send_goal(world_coords)
        else:
            self.get_logger().info('No instance found.')
            self.enable = False
            
    def pixel_to_camera_coords(self, u, v, d):
        X = (u - self.cx) * d / self.fx # fx, cx 값 수정
        Y = (v - self.cy) * d / self.fy # fy, cy 값 수정
        Z = d
        return np.array([X, Y, Z, 1])

    def camera_to_world_coords(self, camera_coords):
        world_coords = np.dot(self.extrinsic_matrix, camera_coords) # extrinsic matrix 수정
        return world_coords[:3]

    def send_goal(self, world_coords):
        goal_msg = MoveToPosition.Goal()
        goal_msg.position.x = world_coords[0]
        goal_msg.position.y = world_coords[1]
        goal_msg.position.z = world_coords[2]

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_state}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

        if self.enable:
            self.get_logger().info('Requesting new images from Depth Camera Node...')
            response = self.send_request()
            self.get_logger().info(f'Received RGB and Depth images {self.image_number}: {response.rgb.header}, {response.depth.header}')
            self.segmentation3d(response.rgb, response.depth)
        else:
            self.get_logger().info('No tomato')

def main(args=None):
    rclpy.init(args=args)  # rclpy 초기화
    node = SegAnd3DNode()  # SegmentationNode 초기화
    response = node.send_request()  # Depth Camera Node에 서비스 요청을 보내고 RGB와 Depth 이미지를 받음
    node.get_logger().info(f'Received RGB and Depth images {node.image_number}: {response.rgb.header}, {response.depth.header}')
    node.segmentation3d(response.rgb, response.depth)  # 받은 이미지를 사용하여 세그멘테이션 수행
    node.get_logger().info('Transmit world coordinates to manipulator!')  # 로그 출력
    rclpy.spin(node)  # 노드를 스핀하여 실행 상태를 유지
    rclpy.shutdown()  # rclpy 종료

if __name__ == '__main__':
    main()
