#!/usr/bin/env python

from __future__ import print_function
import os
import pybullet as p

from pybullet_tools.utils import add_data_path, connect, dump_body, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, LockRenderer, link_from_name, get_link_pose, \
    multiply, Pose, Point, interpolate_poses, HideOutput, draw_pose, set_camera_pose, load_pybullet, \
    assign_link_colors, add_line, point_from_pose, remove_handles, BLUE, INF

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics, check_ik_solver

TRANSLATION = [ 0.06055646, -0.0054715, 0.17928975]
ROTATION = [-0.16294472, 0.90890329, 0.34755264, -0.16294472]
hhh = [[TRANSLATION, ROTATION]] * 2
ARM_CORD = [-0.6, 0, 0]

def test_retraction(robot, info, tool_link, distance=0.1, **kwargs):
    ik_joints = get_ik_joints(robot, info, tool_link)
    start_pose = get_link_pose(robot, tool_link)
    end_pose = multiply(start_pose, Pose(Point(z=-distance)))
    handles = [add_line(point_from_pose(start_pose), point_from_pose(end_pose), color=BLUE)]
    #handles.extend(draw_pose(start_pose))
    #handles.extend(draw_pose(end_pose))
    path = []
    # pose_path = list(interpolate_poses(start_pose, end_pose, pos_step_size=0.01))
    pose_path = list(hhh)

    for i, pose in enumerate(pose_path):
        print(pose)
        print('Waypoint: {}/{}'.format(i+1, len(pose_path)))
        handles.extend(draw_pose(pose))
        conf = next(either_inverse_kinematics(robot, info, tool_link, pose, **kwargs), None)
        print(conf)
        if conf is None:
            print('Failure!')
            path = None
            wait_for_user()
            break
        set_joint_positions(robot, ik_joints, conf)
        path.append(conf)
        wait_for_user()
        # for conf in islice(ikfast_inverse_kinematics(robot, info, tool_link, pose, max_attempts=INF, max_distance=0.5), 1):
        #    set_joint_positions(robot, joints[:len(conf)], conf)
        #    wait_for_user()
    remove_handles(handles)
    return path

def test_ik(robot, info, tool_link, tool_pose):
    draw_pose(tool_pose)
    # TODO: sort by one joint angle
    # TODO: prune based on proximity
    ik_joints = get_ik_joints(robot, info, tool_link)
    for conf in either_inverse_kinematics(robot, info, tool_link, tool_pose, use_pybullet=False,
                                          max_distance=INF, max_time=10, max_candidates=INF):
        # TODO: profile
        set_joint_positions(robot, ik_joints, conf)
        wait_for_user()

def load_gripper_obj(orientation, position):
    mass = 0
    path = "/home/donglin/Github/pytorch_6dof-graspnet/gripper_models/panda_gripper/hand.stl"
    shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName=path,
                                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH |
                                    p.GEOM_CONCAVE_INTERNAL_EDGE
                            )
    p.createMultiBody(mass, -1, shape_id,
                    position, orientation)

#####################################

def main():
    connect(use_gui=True)
    add_data_path()
    draw_pose(Pose(), length=1.)
    set_camera_pose(camera_point=[1, -1, 1])

    plane = p.loadURDF("plane.urdf")
    with LockRenderer():
        with HideOutput(True):
            # robot = load_pybullet(FRANKA_URDF, fixed_base=True)
            robot= p.loadURDF(os.path.join("/home/donglin/Desktop", "franka_panda/panda.urdf"), useFixedBase=True, basePosition=ARM_CORD)
            assign_link_colors(robot, max_colors=3, s=0.5, v=1.)
            #set_all_color(robot, GREEN)
    obstacles = [plane] # TODO: collisions with the ground

    dump_body(robot)
    print('Start?')
    wait_for_user()

    info = PANDA_INFO
    tool_link = link_from_name(robot, 'panda_hand')
    draw_pose(Pose(), parent=robot, parent_link=tool_link)
    joints = get_movable_joints(robot)
    print('Joints', [get_joint_name(robot, joint) for joint in joints])
    check_ik_solver(info)
    load_gripper_obj(ROTATION, TRANSLATION)
    sample_fn = get_sample_fn(robot, joints)
    for i in range(10):
        print('Iteration:', i)
        conf = sample_fn()
        # print(conf)
        set_joint_positions(robot, joints, conf)
        wait_for_user()
        # test_ik(robot, info, tool_link, get_link_pose(robot, tool_link))
        test_retraction(robot, info, tool_link, use_pybullet=True,
                        max_distance=0.1, max_time=0.05, max_candidates=100)
    disconnect()

if __name__ == '__main__':
    main()