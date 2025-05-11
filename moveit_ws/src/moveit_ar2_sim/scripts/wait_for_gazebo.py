
  #!/usr/bin/env python3
  import rospy
  import rostest
  import unittest
  from std_msgs.msg import String

  class TestGazebo(unittest.TestCase):
      def test_gazebo_ready(self):
          rospy.init_node('test_gazebo', anonymous=True)
          timeout = rospy.get_param('~timeout', 30.0)
          start_time = rospy.get_time()
          
          while not rospy.is_shutdown() and rospy.get_time() - start_time < timeout:
              topics = rospy.get_published_topics()
              if any('/gazebo/model_states' in topic for topic in topics):
                  rospy.loginfo("Gazebo is ready!")
                  return
              rospy.sleep(1.0)
          
          self.fail("Timeout waiting for Gazebo to be ready")

  if __name__ == '__main__':
      rostest.rosrun('moveit_ar2_sim', 'wait_for_gazebo', TestGazebo)

