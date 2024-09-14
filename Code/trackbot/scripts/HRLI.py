#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Bool, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from openai import OpenAI
import ast
import subprocess
import cv2
import requests
import base64


client = OpenAI(api_key="")

class HRLI:

    def __init__(self):
        rospy.init_node('HRLI', anonymous=True)
        
        # Publish stop topic
        self.stop_pub = rospy.Publisher('hrl_stop_request', Bool, queue_size=1)

        # Publish linear and angular topic
        self.linear_request_pub = rospy.Publisher('hrl_linear_request', Int32, queue_size=1)
        self.angular_request_pub = rospy.Publisher('hrl_angular_request', Int32, queue_size=1)

        # Publish speed
        self.pub_speed = rospy.Publisher('/speed', Int32, queue_size=1)

        # Get camera frame
        self.cam_feed = rospy.Subscriber('camera_frame', Image, self.image_callback)
        # Request camera frame 
        self.cam_request = rospy.Publisher('request_frame', Int32, queue_size=1)

        # ROS Rate
        self.rate = rospy.Rate(10)

        # Initiate history
        self.history = "conversation started"

        # Initiate cam waiting
        self.cam_waiting = False

        # Initialize CvBridge
        self.bridge = CvBridge()

    def launch_teleop(self):

        #currently not working, window does not appear for teleop
        try:
            print("Attempting to launch teleop node")
            command = 'source ~/catkin_ws/devel/setup.bash && rosrun trackbot teleop.py'
            subprocess.Popen(['tmux', 'new-session', '-d', '-s', 'teleop_session', 'bash -c "{}"'.format(command)])
            print("Teleop node launched")
        except Exception as e:
            print(f"Failed to launch teleop node: {e}")

    # Update the instruction for the Chat AI
    def update_instruction(self,history):
        self.class_inst = f'''
            These instructions are for guiding a robot's actions. You will receive a request and need to classify it into an integer and provide a response and additional information.

            The output must strictly be an array of 4 values: [int, string, int, int]

            The first value of the array is an integer representing the request type:
                - 1: Request to stop the robot from moving
                - 2: Request to move the robot in a simple direction (linear or rotate)
                - 3: Request to move the robot via teleoperation method (using keyboard)
                - 4: Request that can be solved using camera attached to front of robot able to see the real world
                - 5: Request to change the speed of the robot
                - 0: Request does not fit any of the above categories

            The second value of the array is a string response to the user:
                - If the first value is not 0: Inform the user you understood and the process is underway.
                - If the first value is 0 and it's a question or social request: Answer accordingly.
                - If the first value is 0 and it's a specific instruction: Indicate you are unable to perform the action, staying in character/personality.

            if request type 2 is given then:  
                The third value of the array is an int that holds the linear.x value of a Twist message as a distance in cm (negative values indicate moving backwards, positive values indicate moving forwards)
                The fourth value of the array is an int that holds the angular.z value of a Twist message as an angle in degrees (positive values is rotating left, negative values is rotating right)
                Twist message is defined by the ROS geometry_msgs
                
            If the request type is 5:
                The third value is an int from 0 to 100 describing the speed of the robot.
                The fourth value is an int set to 0.

            If the request type is neither 2 nor 5:
                The third and fourth values should both be set to 0.

            Your responses will mimic the personality of GLaDOS from Portal, incorporating occasional sarcasm. Your name is Trackbot, created by Julian Leclerc.

            To summarize:
            The output must strictly be an array with 4 values: [int, string, int, int]
            (The output should be enclosed in [])
            '''
        
            #Everything after this paragraph is a traceback of the current conversation we have been having.
            #This will help for Names and knowoing previous commands or even keep a coherent conversation
            #But please try to not repeat yourself / same sentence structure
            #The memory content is  ordered from new to old, the "trackbot response: " is your message and "user request:" is my message
            #conversation memory:

            #({history})

    def format_message(self, role, content):
        return {"role": role, "content": content}

    def get_response(self, messages):
        completion = client.chat.completions.create(
            model='gpt-3.5-turbo',
            messages=messages,
        )
        content = completion.choices[0].message.content.strip()
        return content
    

    def get_cam_response(self, base64_image):
        self.model_cam = 'gpt-4o'
        self.url_cam = 'https://api.openai.com/v1/chat/completions'
        self.headers_cam = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {client.api_key}"
        }
        self.payload_cam = {
            "model": "gpt-4-vision-preview",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": f"{self.user_cam_request}, answer request with personality of GladOs from Portal"
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            }
                        }
                    ]
                }
            ],
            "max_tokens": 300
        }


    def image_callback(self, msg):
        print(f"Received callback image")
        try:
            if self.cam_waiting:
                # Convert the ROS Image message to OpenCV2 format
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                # Convert the image to JPEG format
                _, img_encoded = cv2.imencode('.jpg', cv_image)
                img_bytes = img_encoded.tobytes()

                # Encode image for ChatGPT
                base64_image = base64.b64encode(img_bytes).decode('utf-8')
                
                self.get_cam_response(base64_image)

                # Send the image to the specified URL
                response = requests.post(self.url_cam, headers=self.headers_cam, json=self.payload_cam)

                # Check the response
                if response.status_code == 200:
                    #rospy.loginfo('Image successfully sent to server')
                    response_json = response.json()
                    content = response_json['choices'][0]['message']['content']
                    rospy.loginfo('Image successfully sent to server')
                    print(f"trackbot >> {content}")
                    
                else:
                    rospy.logerr('Failed to send image to server: %s', response.text)

                self.cam_waiting = False
                print(f"user >> ")
                return

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")
        



    def publish_frames(self):
        while not rospy.is_shutdown():
            user_input = input("user >> ")
            if user_input.lower() == 'exit':
                break
            
            self.update_instruction(self.history)

            messages = [
                self.format_message("system", self.class_inst),
                self.format_message("user", user_input)
            ]
            response = self.get_response(messages)

            try:
                parsed_response = ast.literal_eval(response)
                if isinstance(parsed_response, list) and len(parsed_response) == 4:
                    node_request = parsed_response[0]
                    speech_response = parsed_response[1]
                    a = parsed_response[2]
                    b = parsed_response[3]

                    print(f"trackbot >> {speech_response}")
                    #print(f"debug >> {parsed_response}")

                    
                    self.history = f'''
                    {self.history}
                    > user request: {user_input}
                    > trackbot response: {response}
                    '''

                    if node_request == 1: # stop robot request 
                        self.stop_pub.publish(True)
                        self.linear_request_pub.publish(0)
                        self.angular_request_pub.publish(0)
                    else:
                        self.stop_pub.publish(False)

                    if node_request == 2: # Basic movement request
                        self.linear_request_pub.publish(a)
                        self.angular_request_pub.publish(b)
                    
                    if node_request == 3: # teleop request
                        self.launch_teleop()
                    
                    if node_request == 5: # change robot speed request
                        self.pub_speed.publish(a)

                    if node_request == 4: # look what is in front with camera
                        self.cam_waiting = True
                        self.user_cam_request = user_input
                        self.cam_request.publish(1)

                    else:
                        self.cam_waiting = False

                else:
                    print("Invalid response format:", response)
            except (ValueError, SyntaxError) as e:
                print("Failed to parse response:", e)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = HRLI()
        node.publish_frames()
    except rospy.ROSInterruptException:
        pass
