#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time
import settings

import math
from math import sin, cos, radians, pi


try:
    sys.path.append(glob.glob(settings.CARLA_PATH + f'/PythonAPI/carla/dist/carla-*{sys.version_info.major}.{sys.version_info.minor}-{"win-amd64" if os.name == "nt" else "linux-x86_64"}.egg')[0])
except IndexError:
    pass
# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

_HOST_ = '127.0.0.1'
_PORT_ = 2000
_SLEEP_TIME_ = .1

def angle_of_line(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2-y1, x2-x1))

def angle_difference(sourceA, targetA):
    a = targetA - sourceA
    a = (a + 180) % 360 - 180
    return a

def point_pos(x0, y0, d, theta):
    theta_rad = math.pi/2 - radians(theta)
    return x0 + d*cos(theta_rad), y0 + d*sin(theta_rad)

def get_dist(x1,y1,x2,y2):
    return math.hypot(x2 - x1, y2 - y1)


def main():
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()

    while(True):
        # get camera location
        t = world.get_spectator().get_transform()
        # coordinate_str = "(x,y,z) = ({},{},{}) | (pitch,yaw,roll) = ({},{},{})".format(
        # 	round(t.location.x,3), round(t.location.y,3), round(t.location.z,3),
        # 	round(t.rotation.pitch,3), round(t.rotation.yaw,3), round(t.rotation.roll,3)
        # 	)
        # print (coordinate_str)

        pusatx = -.5
        pusaty = .5
        dis = 21.5
        cam_dir = t.rotation.yaw
        roundabout_to_cam = angle_of_line(0,0,t.location.x, t.location.y)
        angle_diff = angle_difference(roundabout_to_cam, cam_dir)
        dist = get_dist(pusatx,pusaty,t.location.x,t.location.y)
        dist_proc = abs(dist-dis)

        angle_diff_abs = (angle_diff) # range 45 sampe 135
        angle_diff_abs += 90 # range -45 sampe 45
        angle_diff_abs = abs(angle_diff_abs) # range 45 - 0 - 45
        if angle_diff_abs>180:
            angle_diff_abs = abs(-180 + (angle_diff_abs-180))
        # 180-270
        if angle_diff_abs!=0:
            reward = 1/angle_diff_abs
        reward = min(1,reward)
        # reward 1 arah end   ===================================

        # reward 2 jarak start ==================================
        dist_proc = abs(dist-dis)
        reward2 = 1/(dist_proc*10)
        reward2 = min(1,reward2)
        # reward 2 jarak end   ==================================

        # Punishment 1 jarak start ==============================
        punishment = 0
        if dist > 30:
            punishment = .5
        # Punishment 1 jarak end   ==============================

        # reward total start ====================================
        reward_total = reward+reward2-punishment
        # reward total end   ====================================


        # print("cam_dir: ", round(cam_dir,2), ". roundabout to cam: ",
        # 		round(roundabout_to_cam,2), ". angle_diff: ",
        # 		round(angle_diff,2), ". reward: ", round(reward,2),
        # 		". angle_diff_abs: ", round(angle_diff_abs,2),
        # 		". distance: ", get_dist(0,0,t.location.x,t.location.y))

        # print("dist: ",round(dist,2),". dist_proc: ",round(dist_proc,2),". reward2: ",reward2)

        print("reward1:",reward, "reward2:", reward2, "punishment:", punishment, "total:",reward_total)

        # time.sleep(_SLEEP_TIME_)

if __name__ == '__main__':
    main()


