import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from pmt_interfaces.srv import SetBool
from pmt_interfaces.action import MoveToPosition

class Basket(Node):

    def __init__(self):
        super().__init__('basket_node')

        # Service Server
        self.srv = self.create_service(SetBool, 'connect_manipulator', self.handle_connect_manipulator)

        # Action Client
        self.action_client = ActionClient(self, MoveToPosition, 'move_to_position')

        # Service Client for comeback
        self.comeback_client = self.create_client(SetBool, 'comeback')

    def handle_connect_manipulator(self, request, response):
        if request.data:
            self.get_logger().info('Received request to connect manipulator and move to position')
            
            # Sending goal to the action server (pmt_node)
            goal_msg = MoveToPosition.Goal()
            goal_msg.x = 1.0  # basket 현재좌표 저장
            goal_msg.y = 2.0
            goal_msg.z = 3.0
    
            response.success = True
            response.message = 'connect connect_manipulator node and Basket node'
            
            self.send_goal(goal_msg)
        else:
            response.success = False
            response.message = 'no connect connect_manipulator and Basket'
        return response

    def send_goal(self, goal_msg):
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
        self.get_logger().info(f'Received feedback: {feedback.current_state}') # PMT의 현재 위치

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded!')
        
            self.call_comeback_service(True)
        else:
            self.get_logger().info('Goal failed!')

    def call_comeback_service(self, comeback_signal):
        req = SetBool.Request()
        req.data = comeback_signal
        
        self.comeback_client.wait_for_service()
        future = self.comeback_client.call_async(req)
        future.add_done_callback(self.comeback_response_callback)

    def comeback_response_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('Received OK signal from comeback service')
        else:
            self.get_logger().info('Failed to receive OK signal from comeback service')

def main(args=None):
    rclpy.init(args=args)
    node = Basket()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
