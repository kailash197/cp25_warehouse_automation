import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String
from warehouse_automation_pkg.action import Localize
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class GlobalLocalizationClient(Node):
    def __init__(self):
        super().__init__('global_localization_client')
        
        # Use reentrant callback group for proper parallel handling
        self.cb_group = ReentrantCallbackGroup()
        
        # Subscribers
        self.trigger_sub = self.create_subscription(
            Empty, '/wh_glocalization_goal', 
            self.trigger_callback, 10,
            callback_group=self.cb_group)
            
        self.cancel_sub = self.create_subscription(
            Empty, '/wh_glocalization_cancel',
            self.cancel_callback, 10,
            callback_group=self.cb_group)
            
        # Publishers
        self.feedback_pub = self.create_publisher(
            String, '/wh_glocalization_feedback', 10)
            
        self.result_pub = self.create_publisher(
            String, '/wh_glocalization_result', 10)
            
        # Action client with callback group
        self.action_client = ActionClient(
            self, Localize, '/wh_global_localization',
            callback_group=self.cb_group)
            
        self.current_goal_handle = None
        self.lock = threading.Lock()  # For thread-safe goal handle access

    def trigger_callback(self, msg):
        self.get_logger().info('Received trigger, waiting for action server...')
        
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return
            
        goal_msg = Localize.Goal()
        self.get_logger().info('Sending localization goal...')
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
            
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        with self.lock:
            try:
                self.current_goal_handle = future.result()
                
                if not self.current_goal_handle.accepted:
                    self.get_logger().warn('Goal rejected by server')
                    self.current_goal_handle = None
                    return
                    
                self.get_logger().info('Goal accepted with ID: %r' % 
                                     self.current_goal_handle.goal_id.uuid)
                
                # Register result callback
                result_future = self.current_goal_handle.get_result_async()
                result_future.add_done_callback(self.result_callback)
                
            except Exception as e:
                self.get_logger().error(f'Error in goal response: {str(e)}')
                self.current_goal_handle = None

    def cancel_callback(self, msg):
        with self.lock:
            if self.current_goal_handle is None:
                self.get_logger().warn('No active goal to cancel')
                return
                
            if not self.current_goal_handle.status == GoalStatus.STATUS_ACCEPTED:
                self.get_logger().warn('Goal is not in a cancelable state')
                return
                
            self.get_logger().info('Requesting goal cancellation...')
            
            # Create new future for cancellation
            cancel_future = self.action_client._cancel_goal_async(self.current_goal_handle)
            cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        try:
            cancel_response = future.result()
            
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Cancel request accepted for goal ID: %r' % 
                                     cancel_response.goals_canceling[0].goal_id.uuid)
            else:
                self.get_logger().warn('Cancel request rejected')
                
        except Exception as e:
            self.get_logger().error(f'Error in cancel response: {str(e)}')
        finally:
            with self.lock:
                self.current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        msg = String()
        msg.data = (f'cov_x: {feedback.cov_x}, cov_y: {feedback.cov_y}, '
                   f'cov_yaw: {feedback.cov_yaw}, status: {feedback.status}')
                   
        self.feedback_pub.publish(msg)
        self.get_logger().info(f'Feedback: {msg.data}', throttle_duration_sec=1.0)

    def result_callback(self, future):
        try:
            result = future.result()
            msg = String()
            msg.data = f'success: {result.result.success}, status: {result.result.status}'
            
            self.result_pub.publish(msg)
            self.get_logger().info(f'Final result: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Error in result: {str(e)}')
        finally:
            with self.lock:
                self.current_