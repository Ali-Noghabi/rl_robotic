# quadruped_simulation.py
import pybullet as p
import time
import rospy
from sensor_msgs.msg import JointState

def main():
    # Initialize ROS node
    rospy.init_node('quadruped_simulation_publisher')
    pub = rospy.Publisher('/quadruped/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version
    p.setGravity(0, 0, -9.81)
    planeId = p.loadURDF("urdf/plane.urdf")
    startPos = [0, 0, 0.5]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    quadruped = p.loadURDF("urdf/anymal.urdf", startPos, startOrientation)

    joint_names = [f'joint_{i}' for i in range(p.getNumJoints(quadruped))]
    
    while not rospy.is_shutdown():
        p.stepSimulation()
        joint_states = JointState()
        joint_states.header.stamp = rospy.Time.now()
        joint_states.name = joint_names
        joint_states.position = []
        joint_states.velocity = []
        joint_states.effort = []

        for joint_index in range(p.getNumJoints(quadruped)):
            joint_info = p.getJointState(quadruped, joint_index)
            joint_states.position.append(joint_info[0])
            joint_states.velocity.append(joint_info[1])
            joint_states.effort.append(joint_info[2])

        pub.publish(joint_states)
        rate.sleep()

    p.disconnect()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
