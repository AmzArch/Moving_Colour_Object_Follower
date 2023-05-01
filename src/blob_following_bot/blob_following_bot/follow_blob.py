import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from blob_interfaces.msg import BlobsList, Blob

class BlobFollower(Node):

    def __init__(self):
        super().__init__('blob_follower')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.blobs_sub = self.create_subscription(BlobsList, '/detected_blobs', self.blobs_callback, 10)
        self.color = 'green'   # choose the desired color of the blob to follow
        self.prev_area = 0     # initialize previous area to zero

    def blobs_callback(self, msg):
        largest_blob = None
        for blob in msg.blobs:
            if blob.color == self.color:
                if largest_blob is None or blob.area > largest_blob.area:
                    largest_blob = blob
        if largest_blob is not None:
            self.follow_blob(largest_blob)

    def follow_blob(self, blob):
        twist = Twist()
        
        '''
        The constant value of 480*640 represents the total number of pixels in the image captured by my webcam.
        Change this value to match the resolution of your own webcam.
        You can also adjust the multiplier value as needed.
        '''
        
        
        # check if area is decreasing or increasing
        if blob.area < self.prev_area:
            speed = 2.0*blob.area/(480*640)   # increase speed if area is decreasing
        else:
            speed = 1.0*blob.area/(480*640)   # decrease speed if area is increasing
        twist.linear.x = speed
        if blob.x < 240:  # if blob is on the left side of the image
            twist.angular.z = 0.5  # turn left
        else:
            twist.angular.z = -0.5  # turn right
        self.cmd_vel_pub.publish(twist)
        self.prev_area = blob.area   # update previous area to current area

def main(args=None):
    rclpy.init(args=args)
    blob_follower = BlobFollower()
    rclpy.spin(blob_follower)
    blob_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
