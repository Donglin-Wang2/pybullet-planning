import pybullet as p
import os
from pybullet_tools.utils import add_data_path, connect, dump_body, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, LockRenderer, link_from_name, get_link_pose, \
    multiply, Pose, Point, interpolate_poses, HideOutput, draw_pose, set_camera_pose, load_pybullet, \
    assign_link_colors, add_line, point_from_pose, remove_handles, BLUE, INF

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics

POSITION = [ 0.06055646, -0.0054715, 0.17928975]
ORIENTATION = [-0.16294472, 0.90890329, 0.34755264, -0.16294472]
POSE = [POSITION, ORIENTATION]
ARM_CORD = [-0.5, 0, 0]

connect(use_gui=True)
set_camera_pose(camera_point=[1, -1, 1])

def load_gripper_obj(orientation, position):
    mass = 0
    path = "/home/donglin/Github/pytorch_6dof-graspnet/gripper_models/panda_gripper/hand.stl"
    shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName=path,
                                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH |
                                    p.GEOM_CONCAVE_INTERNAL_EDGE
                            )
    p.createMultiBody(mass, -1, shape_id, position, orientation)



robot= p.loadURDF(os.path.join("/home/donglin/Desktop", "franka_panda/panda.urdf"), useFixedBase=True, basePosition=ARM_CORD)
assign_link_colors(robot, max_colors=3, s=0.5, v=1.)

tool_link = link_from_name(robot, 'panda_hand')
print(tool_link, type(tool_link))
ik_joints = get_ik_joints(robot, PANDA_INFO, tool_link)
start_pose = get_link_pose(robot, tool_link)
pose_path = list(interpolate_poses(start_pose, POSE, pos_step_size=0.01))
conf = next(either_inverse_kinematics(robot, PANDA_INFO, tool_link, POSE), None)
print(conf)
load_gripper_obj(ORIENTATION, POSITION)
# p.setJointMotorControlArray(robot, ik_joints, p.POSITION_CONTROL, targetPositions=conf)
set_joint_positions(robot, ik_joints, conf)
link_state = p.getLinkState(robot, tool_link, computeForwardKinematics=True)
print([len(ele) for ele in link_state])
print(link_state[-2], link_state[-1])

# for i, pose in enumerate(pose_path):
#     print("Waypoint {}/{}".format(i+1, len(pose_path)))
#     conf = next(either_inverse_kinematics(robot, PANDA_INFO, tool_link, pose), None)
#     # print(conf, type(conf))
#     # print(ik_joints, type(ik_joints))
#     if conf is None:
#         print("Failure")
#         exit()
#     p.setJointMotorControlArray(robot, ik_joints, p.POSITION_CONTROL, targetPositions=conf)

while (p.isConnected()):
    p.stepSimulation()