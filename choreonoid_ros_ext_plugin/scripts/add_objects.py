from cnoid.Base import *
from cnoid.BodyPlugin import *
from cnoid.PythonSimScriptPlugin import *
import cnoid.Body

import math
import os
import sys
import yaml

import re
import subprocess ## subprocess is recommended but choreonoid crashed when calling rospack find
#import commands

try:
    objs_yaml = os.environ['EXTRA_CHOREONOID_OBJS']
except:
    print("environment variable 'EXTRA_CHOREONOID_OBJS' is not found",  file=sys.stderr)
    raise

def parse_filename(filestr):
    re_find = re.compile(r'\$\(find ([^ ]+)\)')
    ret = re_find.search(filestr)
    if ret != None:
        pkgname = ret.group(1)
        #packagepath = commands.getoutput('rospack find %s'%(pkgname))
        packagepath = subprocess.check_output(['rospack', 'find', pkgname]).decode()
        packagepath = packagepath.rstrip('\n')
        filestr = filestr[:ret.start(0)] + packagepath + filestr[ret.end(0):]

    return filestr

try:
    f = open(objs_yaml, 'r')
    dict_objs = yaml.load(f, Loader=yaml.SafeLoader)
    f.close()
except:
    print("can not read %s"%(objs_yaml), file=sys.stderr)
    raise

sci_path = os.path.abspath(os.path.dirname(__file__))

itemTreeView = ItemTreeView.instance

rootItem = RootItem.instance

world = rootItem.findItem("World")
if world:
    for obj_name in dict_objs:
        obj_info = dict_objs[obj_name]
        if 'file' in obj_info:
            filename = parse_filename(obj_info['file'])
        else:
            continue

        if 'name' in obj_info:
            objname = obj_info['name']
        else:
            objname = obj_name

        if filename[0] != '/' and filename[0] != '$':
            filename = "%s/%s"%(sci_path, filename)

        if not os.path.exists(filename):
            print("file: %s not found"%(filename), file=sys.stderr)
            raise

        robotItem = BodyItem()
        robotItem.load(filename)
        robotItem.setName(objname)
        robot = robotItem.body

        robot_rootLink = robot.rootLink

        if 'static' in obj_info:
            static = obj_info['static']
            if static:
                robot_rootLink.setJointType(cnoid.Body.Link.JointType.FIXED_JOINT)
                robot.updateLinkTree()
            else:
                robot_rootLink.setJointType(cnoid.Body.Link.JointType.FREE_JOINT)
                robot.updateLinkTree()

        if 'static_joint' in obj_info:
            static_joint = obj_info['static_joint']
            if static_joint:
                for i in range(robot.numAllJoints):
                    robot.joint(i).setJointType(cnoid.Body.Link.JointType.FIXED_JOINT)
                robot.updateLinkTree()

        if 'static_joints' in obj_info:
            static_joints = obj_info['static_joints']
            for j in static_joints:
                if robot.link(j):
                    robot.link(j).setJointType(cnoid.Body.Link.JointType.FIXED_JOINT)
                else:
                    print("joint: %s not found"%(j), file=sys.stderr)
            robot.updateLinkTree()

        if 'translation' in obj_info:
            trans = obj_info['translation']
            robot_rootLink.setTranslation(trans);

        if 'rotation' in obj_info:
            rot = obj_info['rotation']
            robot_rootLink.setRotation(rot);

        for i in range(robot.numJoints):
            robot.joint(i).q = 0

        robot.calcForwardKinematics()
        robotItem.storeInitialState()
        world.insertChildItem(robotItem, world.childItem)
        itemTreeView.checkItem(robotItem)
