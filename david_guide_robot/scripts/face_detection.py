import cv2
import rospy
from geometry_msgs.msg import Twist
from datetime import datetime, timedelta
from std_msgs import String

face_classifier = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)

video_capture = cv2.VideoCapture(0)

def detect_bounding_box(vid):
    gray_image = cv2.cvtColor(vid, cv2.COLOR_BGR2GRAY)
    faces = face_classifier.detectMultiScale(gray_image, 1.1, 5, minSize = (40, 40))
    for (x, y, w, h) in faces:
        cv2.rectangle(vid, (x, y), (x+w, y+h), (0, 255, 0), 4)

    return faces

vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size = 10)
rospy.init_node("face_detect")
r = rospy.Rate(5)

not_detect = timedelta()
start_time = datetime.now()

recovery_mode = False

robot_state = rospy.Subscriber("/state", String, queue_size = 10)

while robot_state == "idle":

    result, video_frame = video_capture.read()
    if result is False:
        break

    faces = detect_bounding_box(video_frame)

    if faces == ():
        not_detect = not_detect + datetime.now() - start_time
        start_time = datetime.now()
        if not_detect.total_seconds() >= 3:
            recovery_mode = True

    if recovery_mode == True:
        print("recovery")
        msg = Twist()
        msg.angular.z = 0.5
        vel_pub.publish(msg)

    if faces != ():

        recovery_mode = False
        not_detect = timedelta()

        if (faces[0][0] + 0.5*faces[0][2]) <= 280:
            print("left")
            msg = Twist()
            msg.angular.z = 0.5
            vel_pub.publish(msg)
            faces == ()
        elif(faces[0][0] + 0.5*faces[0][2]) >= 360:
            print("right")
            msg = Twist()
            msg.angular.z = -0.5
            vel_pub.publish(msg)
            faces == ()
        else:
            print("center")
            msg = Twist()
            vel_pub.publish(msg)
            faces == ()

    cv2.imshow(
        "face_detect", video_frame
    )

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video_capture.release()
cv2.destroyAllWindows()