#!/usr/bin/env python
import rospy, sys, os, time
import string
import warnings
import re
import utm
import OsmParser 

from gazebo_ros import gazebo_interface
from rospkg import RosPack

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench
import tf.transformations as tft

model_database_template = """<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://MODEL_NAME</uri>
    </include>
  </world>
</sdf>"""

def usage():
    print('''Commands:
    -[urdf|sdf|trimesh|gazebo] - specify incoming xml is urdf, sdf or trimesh format. gazebo arg is deprecated in ROS Hydro
    -[file|param|database] [<file_name>|<param_name>|<model_name>] - source of the model xml or the trimesh file
    -model <model_name> - name of the model to be spawned.
    -reference_frame <entity_name> - optinal: name of the model/body where initial pose is defined.
                                     If left empty or specified as "world", gazebo world frame is used.
    -gazebo_namespace <gazebo ros_namespace> - optional: ROS namespace of gazebo offered ROS interfaces.  Defaults to /gazebo/ (e.g. /gazebo/spawn_model).
    -robot_namespace <robot ros_namespace> - optional: change ROS namespace of gazebo-plugins.
    -unpause - optional: !!!Experimental!!! unpause physics after spawning model
    -wait - optional: !!!Experimental!!! wait for model to exist
    -trimesh_mass <mass in kg> - required if -trimesh is used: linear mass
    -trimesh_ixx <moment of inertia in kg*m^2> - required if -trimesh is used: moment of inertia about x-axis
    -trimesh_iyy <moment of inertia in kg*m^2> - required if -trimesh is used: moment of inertia about y-axis
    -trimesh_izz <moment of inertia in kg*m^2> - required if -trimesh is used: moment of inertia about z-axis
    -trimesh_gravity <bool> - required if -trimesh is used: gravity turned on for this trimesh model
    -trimesh_material <material name as a string> - required if -trimesh is used: E.g. Gazebo/Blue
    -trimesh_name <link name as a string> - required if -trimesh is used: name of the link containing the trimesh
    -x <x in meters> - optional: initial pose, use 0 if left out
    -y <y in meters> - optional: initial pose, use 0 if left out
    -z <z in meters> - optional: initial pose, use 0 if left out
    -R <roll in radians> - optional: initial pose, use 0 if left out
    -P <pitch in radians> - optional: initial pose, use 0 if left out
    -Y <yaw in radians> - optional: initial pose, use 0 if left out
    -J <joint_name joint_position> - optional: initialize the specified joint at the specified value
    -package_to_model - optional: convert urdf <mesh filename="package://..." to <mesh filename="model://..."
    -b - optional: bond to gazebo and delete the model when this program is interrupted
    ''')
    sys.exit(1)

class SpawnModel():
    def __init__(self):
        self.file_name               = ""
        self.gazebo_namespace        = "/gazebo"
        self.coordinate              = "ENU"
        self.osmFile                 = ""
        self.sdfFile                 = ""
        self.build_world             = False
        self.build_option            = "-a"

    def parseUserInputs(self):
        # get goal from commandline
        for i in range(0,len(sys.argv)):
          if sys.argv[i] == '-file':
            if len(sys.argv) > i+1:
                self.file_name = sys.argv[i+1]
          if sys.argv[i] == '-reference_frame':
            if len(sys.argv) > i+1:
              self.coordinate = sys.argv[i+1]
          if sys.argv[i] == '-namespace':
            if len(sys.argv) > i+1:
              self.gazebo_namespace = sys.argv[i+1]
          if sys.argv[i] == '-gazebo_namespace':
            if len(sys.argv) > i+1:
              self.gazebo_namespace = sys.argv[i+1]
          if sys.argv[i] == '-osmFile':
            self.osmFile = sys.argv[i+1]
          if sys.argv[i] == '-build_world':
            self.build_world = True
          if sys.argv[i] == '-build_option':
            self.build_option = sys.argv[i+1]
          if sys.argv[i] == '-sdf_file':
            self.sdfFile = sys.argv[i+1]

        if not self.file_name and self.build_world:
          rospy.logerr("Error: you must specify the sdf file to build world")
          sys.exit(0)
        
        if self.build_world and not self.build_option:
          rospy.logwwarn("WARNING: will use default build option")
          self.build_option = "-a"

    def callSpawnService(self):

        # wait for model to exist

        if self.file_name != "":
          rospy.loginfo("Loading model XML from file")
          if os.path.exists(self.file_name):
            if os.path.isdir(self.file_name):
              rospy.logerr("Error: file name is a path? %s", self.file_name)
              sys.exit(0)
            if not os.path.isfile(self.file_name):
              rospy.logerr("Error: unable to open file %s", self.file_name)
              sys.exit(0)
          else:
            rospy.logerr("Error: file does not exist %s", self.file_name)
            sys.exit(0)
          # load file
          f = open(self.file_name,'r')
          model_xml = f.read()
          if model_xml == "":
            rospy.logerr("Error: file is empty %s", self.file_name)
            sys.exit(0)

        else:
          rospy.logerr("Error: user specified filename is an empty string")
          sys.exit(0)

        initial_pose = Pose()

        # convert rpy to quaternion for Pose message
        tmpq = tft.quaternion_from_euler(0,0,0)
        q = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])
        initial_pose.orientation = q

        
        success = gazebo_interface.spawn_sdf_model_client('EIU', model_xml, 'world',
                                                            initial_pose, 'map', self.gazebo_namespace)

        return


    def callDeleteService(self):
        try:
            delete_model = rospy.ServiceProxy('%s/delete_model'%(self.gazebo_namespace), DeleteModel)
            delete_model(model_name=self.model_name)
        except rospy.ServiceException as e:
            rospy.logerr("Delete model service call failed: %s", e)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(usage())
    else:
        print("SpawnModel script started") # make this a print incase roscore has not been started
        sm = SpawnModel()
        sm.parseUserInputs()
        sm.callSpawnService()