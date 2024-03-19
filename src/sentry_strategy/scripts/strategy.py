#!/usr/bin/python3
# coding=utf8

import itertools
import threading
import time
from typing import List

import rospy
import actionlib
# 使用 actionlib 机制发送目标，获取进度反馈
# Reference: http://wiki.ros.org/actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from config.nav_points import *

ok_ = False

WAIT_TIME = 15

class RobotController:
    def __init__(self, points):
        rospy.init_node('robot_controller', anonymous=True)

        # actionlib
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.goalplace = None
        self.goals = points
        self.goals_iter = itertools.cycle(self.goals)
        self.current_goal = None
        self.seq_ = 1
        self.seq_last_ = 1

        self.begin = False            #比赛开始

        # 状态跳转函数表
        self.jump = {}
        self.jump.setdefault(lambda _: None)
        init_jump_table(self.jump)

        # 通信变量
        # self.allow_aim_ = False
        # self.allow_nav_ = False
        # self.allow_hit_back_ = True
        self.rotate_ = False
        self.health_ = None
        self.time_ = None
        self.bulletNum_ = None
        self.post_ = None
        self.danger = False

        # 通信线程
        self.worker_thread = threading.Thread(target=self.daemon_)
        self.worker_thread.daemon = True
        self.worker_thread.start()

        while not rospy.is_shutdown() and not ok_:
            rospy.loginfo("[Init] Not ok")
            time.sleep(2)
        self.client.cancel_all_goals()

    def run(self):
        while not rospy.is_shutdown():
            # if self.state == State.WAIT:
            #     self.allow_nav_ = False
            #     if isinstance(self.remainTime_, int) and self.remainTime_ < 600:
            #         # 新建导航点
            #         self.current_goal = next(self.goals_iter)
            #         self.changeState(State.NAV_FORWARD)
            # elif self.state == State.NAV_FORWARD or self.state == State.NAV_HOME:  # 导航或回家
            #     self.run_nav()
            # elif self.state == State.OUT_OF_BULLET:
            #     self.run_out_of_bullet()
            self.run_nav()
            # self.checkInterrupt()

    def changeState(self, index):
        # 取消所有目标
        self.client.cancel_all_goals()
        self.seq_ += 1
        # 切换
        self.goalplace = index

    """
    导航与回家
    """

    def run_nav(self):
        # self.allow_nav_ = True
        # self.allow_aim_ = True if self.state == State.NAV_FORWARD else False
        # self.allow_hit_back_ = True if self.health_ > 400 else False

        '''
        // 1 处
         中断返回若状态切换导致self.seq_ > self.seq_last_后
            重新执行到这里，还是发送旧导航点，
        如果导航结束或超时，self.changeState(self.state)
            导致self.seq_ > self.seq_last，新导航点后在此处发布
        '''
        if self.seq_ > self.seq_last_ :
            # 发送导航点
            self.seq_last_ = self.seq_
            rospy.loginfo(" Send Goal...")
            self.send_goal_(self.goalplace)
            return
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            self.checkInterrupt()
            # //1 中断发生
            if self.seq_ > self.seq_last_:
                return
            # 导航结束
            if self.client.get_state() in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED]:
                # self.client.cancel_goal()
                rospy.loginfo("Nav Done")
                time.sleep(3)
            # 导航超时
            elif (rospy.Time.now() - start_time).to_sec() > WAIT_TIME:  # 如果导航时间超过WAIT_TIME秒
                # self.client.cancel_goal()  # 取消导航
                rospy.loginfo("time out")
                return
            # 仍在导航中
            time.sleep(0.02)

    """
    没子弹，走到角落防止被打到
    """

    def run_out_of_bullet(self):
        while True:
            pass

    # 检查中断，跳转状态或其他处理
    def checkInterrupt(self):
        if self.time_ >= 420:
            # 比赛未开始
            if self.goalplace == Place.no_place:
                return
            self.changeState(Place.no_place)
            rospy.loginfo("not begin")
            return
        if self.time_ <= 419 and self.goalplace == Place.no_place:
            # 比赛开始，没有标点，直接出门去大资源岛
            self.changeState(Place.blue_island)
            rospy.loginfo(" to blue island")
            return
        if self.post_ <= 200 and self.goalplace != Place.blue_home and self.danger == False:
            # 前哨站血量很低，danger 置 true，回家
            self.changeState(Place.blue_home)
            self.danger = True
            rospy.loginfo(" to blue home")
            return
        if self.danger == True and self.time_ < 300 and self.goalplace != Place.blue_island:
            # 前哨站掉了，怎么又自己跑到蓝资源岛了？
            self.changeState(Place.blue_island)
            rospy.loginfo(" to blue island")
            return
        if self.danger == True and self.health_ >= 200 and self.time_ >= 300 and self.goalplace != Place.blue_home:
            # 有血有时间，但是要回家，怪怪的
            self.changeState(Place.blue_home)
            rospy.loginfo(" to blue home")
            return
        if self.danger == True and self.health_ < 200 and self.time_ >= 300 and self.goalplace != Place.blue_supply:
            # 回蓝色补给区逃命
            self.changeState(Place.blue_supply)
            rospy.loginfo(" to blue supply")
            return
        


    def daemon_(self):
        global ok_
        while not rospy.is_shutdown():
            # 上位机控制线程
            # setparam("RobotControl/allow_aim", self.allow_aim_)
            # setparam("RobotControl/allow_nav", self.allow_nav_)
            # 给电控发的 允许小陀螺 和 允许反击
            setparam("RobotControl/allow_rotate", self.rotate_)
            # setparam("RobotControl/allow_hitBack", self.allow_hit_back_)
            ok_ = getparam("RobotStatus/ok", default=None)
            self.health_ = getparam("RobotStatus/hp", default=600)
            self.time_ = getparam("RobotStatus/time", default=600)
            self.post_ = getparam("RobotStatus/post",default = 1500)
            self.bulletNum_ = getparam("RobotControl/bullet", default=700)
            # sleep(0.02)

    # 创建一个 MoveBaseGoal,发送给 MoveBaseServer
    def send_goal_(self, index):
        if index == Place.no_place:
            return
        goal = MoveBaseGoal()
        self.current_goal = self.goals[int(index)]
        goal.target_pose.pose = self.current_goal
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration(0.2)
        self.client.send_goal(goal)
        print(self.current_goal)

def getparam(param, **kwargs):
    return rospy.get_param(param, **kwargs)


def setparam(param, value):
    rospy.set_param(param, value)


def init_jump_table(jump_table):
    # def WAIT2NAV_FORWARD(self):
    #     rospy.loginfo("[Jump] From WAIT Switch to NAV_FORWARD")

    # jump_table[(State.WAIT, State.NAV_FORWARD)] = WAIT2NAV_FORWARD
    return


if __name__ == '__main__':
    points = [Pose(Point(a, b, c), Quaternion(d, e, f, g)) for (a, b, c, d, e, f, g) in points]
    controller = RobotController(points)
    rospy.loginfo("main here")
    controller.run()
