import os
import yaml
from pi import *
from urdf_parser_py.urdf import URDF, Joint, Link, Inertial, Inertia
import rospkg
import xacro
from rosgraph.names import load_mappings
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy as np
from scipy.optimize import minimize


def find_child_link(parent_link, joints_visit, joints):
    for joint in joints:
        if joint.parent == parent_link:
            if joint.name in joints_visit:
                joints_visit.remove(joint.name)
            child_link = find_child_link(joint.child, joints_visit, joints)
            if not joints_visit:
                return child_link
    return parent_link


def tune(path=os.getcwd(), load=0, pattern=""):
    for root, dirs, files in os.walk(path):
        if pattern in root:
            for file in files:
                if file.endswith(".setup_assistant"):
                    with open(os.path.join(root, file), "r") as stream:
                        setup_assistant = yaml.safe_load(stream)

                    with open(
                        os.path.join(root, "config", "gazebo_controllers.yaml"), "r"
                    ) as stream:
                        gazebo_controllers = yaml.safe_load(stream)

                    ros_controllers_file = os.path.join(
                        root, "config", "ros_controllers.yaml"
                    )

                    with open(ros_controllers_file, "r") as stream:
                        ros_controllers = yaml.safe_load(stream)

                    publish_rate = gazebo_controllers["joint_state_controller"][
                        "publish_rate"
                    ]

                    urdf = setup_assistant["moveit_setup_assistant_config"]["URDF"]
                    package_path = rospkg.RosPack().get_path(urdf["package"])
                    xacro_file = os.path.join(package_path, urdf["relative_path"])
                    mappings = load_mappings(urdf["xacro_args"].split(" "))
                    urdf_doc = xacro.process_file(xacro_file, mappings=mappings)
                    urdf_string = urdf_doc.toxml()
                    robot = URDF.from_xml_string(urdf_string)

                    base_link = robot.get_root()
                    joints_controlled = sum(
                        [
                            list(controller_value["gains"].keys())
                            for controller_value in ros_controllers.values()
                        ],
                        [],
                    )
                    joint_tabs = len(max(joints_controlled, key=len)) + 4
                    end_link = find_child_link(
                        base_link, joints_controlled, robot.joints
                    )
                    kdl_kin = [KDLKinematics(robot, base_link, end_link)]

                    robot.add_link(Link("load", inertial=Inertial(load, Inertia())))
                    robot.add_joint(Joint("load_joint", end_link, "load", "fixed"))
                    end_link = "load"
                    kdl_kin.append(KDLKinematics(robot, base_link, end_link))

                    joint_names = kdl_kin[0].get_joint_names()
                    q = [[0] * len(joint_names)] * 2
                    m = [0] * 2
                    bounds = [
                        tuple(bound)
                        for bound in np.transpose(kdl_kin[0].get_joint_limits())
                    ]

                    for controller_key, controller_value in ros_controllers.items():
                        controller_value["constraints"] = {
                            "stopped_velocity_tolerance": 0
                        }

                        if publish_rate == 50:
                            controller_value.pop("state_publish_rate", None)
                        else:
                            controller_value["state_publish_rate"] = publish_rate

                        for joint_key, joint_value in controller_value["gains"].items():
                            print((joint_key + "\t").expandtabs(joint_tabs), end="")

                            joint = robot.joint_map[joint_key]
                            limit = joint.limit
                            f = limit.effort
                            v = limit.velocity
                            # s = abs(limit.upper - limit.lower)
                            d = f / v

                            index = joint_names.index(joint_key)

                            for i in range(2):

                                def mq(q):
                                    (-1) ** i * kdl_kin[i].inertia(q)[index, index]

                                res = minimize(mq, q[i], bounds=bounds)
                                q[i] = res.x
                                m[i] = float((-1) ** i * res.fun)

                            # print(*m, sep="\t", end="\t")

                            p, i, *x = pi(m[1], d, 1 / publish_rate)
                            print(p, i, *x, sep="\t")

                            joint_value["p"] = p
                            joint_value["i"] = i
                            joint_value["d"] = 0 * d
                            joint_value["i_clamp"] = f

                    with open(ros_controllers_file, "w") as stream:
                        yaml.dump(ros_controllers, stream)


if __name__ == "__main__":
    path = os.path.join(os.getcwd(), "src", "moveit_config")
    tune(path, 0, "hande_d435i")
    # tune(path, 1, "coactegpc")
