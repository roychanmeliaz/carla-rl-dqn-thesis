#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


from print_rewards import angle_difference, angle_of_line
from generate_waypoint import get_dist
import glob
import os
import sys
import time
import settings

import csv

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

bunderan_waypoints = []
bunderan_waypoints.append((103.84294891357422,64.19479370117188))
bunderan_waypoints.append((102.77753448486328,64.27484893798828))
bunderan_waypoints.append((101.66400909423828,64.37203216552734))
bunderan_waypoints.append((100.55635833740234,64.50273132324219))
bunderan_waypoints.append((99.46969604492188,64.66295623779297))
bunderan_waypoints.append((98.37799835205078,64.8742446899414))
bunderan_waypoints.append((97.33356475830078,65.15473175048828))
bunderan_waypoints.append((96.27842712402344,65.4795913696289))
bunderan_waypoints.append((95.25698852539062,65.83106994628906))
bunderan_waypoints.append((94.32801818847656,66.30314636230469))
bunderan_waypoints.append((93.40272521972656,66.88826751708984))
bunderan_waypoints.append((92.5385513305664,67.50526428222656))
bunderan_waypoints.append((91.66393280029297,68.0943374633789))
bunderan_waypoints.append((90.75911712646484,68.6967544555664))
bunderan_waypoints.append((89.87803649902344,69.27584075927734))
bunderan_waypoints.append((88.99041748046875,69.86024475097656))
bunderan_waypoints.append((88.12064361572266,70.43084716796875))
bunderan_waypoints.append((87.24790954589844,70.99837493896484))
bunderan_waypoints.append((86.38330841064453,71.55154418945312))
bunderan_waypoints.append((85.47853088378906,72.11196899414062))
bunderan_waypoints.append((84.56084442138672,72.51951599121094))
bunderan_waypoints.append((83.5077896118164,72.98719024658203))
bunderan_waypoints.append((82.542724609375,73.41484069824219))
bunderan_waypoints.append((81.54732513427734,73.75298309326172))
bunderan_waypoints.append((80.48033905029297,74.00922393798828))
bunderan_waypoints.append((79.3795166015625,74.19007873535156))
bunderan_waypoints.append((78.2535629272461,74.29595947265625))
bunderan_waypoints.append((77.13347625732422,74.296630859375))
bunderan_waypoints.append((76.00569915771484,74.26954650878906))
bunderan_waypoints.append((74.89289093017578,74.21240997314453))
bunderan_waypoints.append((73.77533721923828,74.146484375))
bunderan_waypoints.append((72.63985443115234,74.05473327636719))
bunderan_waypoints.append((71.52111053466797,73.77349090576172))
bunderan_waypoints.append((70.51206970214844,73.35880279541016))
bunderan_waypoints.append((69.4709701538086,72.8286361694336))
bunderan_waypoints.append((68.51145935058594,72.21977996826172))
bunderan_waypoints.append((67.57743072509766,71.55890655517578))
bunderan_waypoints.append((66.62791442871094,70.85128784179688))
bunderan_waypoints.append((65.80152130126953,70.05152130126953))
bunderan_waypoints.append((65.04995727539062,69.16112518310547))
bunderan_waypoints.append((64.35430908203125,68.22051239013672))
bunderan_waypoints.append((63.70584487915039,67.26461029052734))
bunderan_waypoints.append((63.11410140991211,66.27951049804688))
bunderan_waypoints.append((62.57427215576172,65.25041198730469))
bunderan_waypoints.append((62.11251449584961,64.18560028076172))
bunderan_waypoints.append((61.72903823852539,62.99799728393555))
bunderan_waypoints.append((61.51875686645508,61.87117385864258))
bunderan_waypoints.append((61.36729431152344,60.71494674682617))
bunderan_waypoints.append((61.347434997558594,59.61738967895508))
bunderan_waypoints.append((61.43709945678711,58.53907012939453))
bunderan_waypoints.append((61.59947204589844,57.25206756591797))
bunderan_waypoints.append((61.82103729248047,56.13203048706055))
bunderan_waypoints.append((62.13115692138672,55.03913116455078))
bunderan_waypoints.append((62.5803108215332,53.9842414855957))
bunderan_waypoints.append((63.13450622558594,52.988460540771484))
bunderan_waypoints.append((63.76716232299805,52.05413055419922))
bunderan_waypoints.append((64.50353240966797,51.17920684814453))
bunderan_waypoints.append((65.36164093017578,50.44059371948242))
bunderan_waypoints.append((66.2762222290039,49.80142593383789))
bunderan_waypoints.append((67.1892318725586,49.164451599121094))
bunderan_waypoints.append((68.1085205078125,48.523651123046875))
bunderan_waypoints.append((69.082763671875,47.9354248046875))
bunderan_waypoints.append((70.12930297851562,47.44189453125))
bunderan_waypoints.append((71.28501892089844,46.98786163330078))
bunderan_waypoints.append((72.61466217041016,46.534759521484375))
bunderan_waypoints.append((73.71404266357422,46.32102584838867))
bunderan_waypoints.append((74.83573913574219,46.166439056396484))
bunderan_waypoints.append((75.9391860961914,46.07291793823242))
bunderan_waypoints.append((77.06755828857422,46.02342224121094))
bunderan_waypoints.append((78.1922607421875,46.03863525390625))
bunderan_waypoints.append((79.2781753540039,46.15121078491211))
bunderan_waypoints.append((80.39747619628906,46.31387710571289))
bunderan_waypoints.append((81.49624633789062,46.56361770629883))
bunderan_waypoints.append((82.53202056884766,46.96931076049805))
bunderan_waypoints.append((83.532958984375,47.422508239746094))
bunderan_waypoints.append((84.5123062133789,47.935237884521484))
bunderan_waypoints.append((85.44886779785156,48.50223159790039))
bunderan_waypoints.append((86.36940002441406,49.06724548339844))
bunderan_waypoints.append((87.34111785888672,49.665287017822266))
bunderan_waypoints.append((88.28600311279297,50.27983474731445))
bunderan_waypoints.append((89.1908950805664,50.90321350097656))
bunderan_waypoints.append((90.12358093261719,51.522972106933594))
bunderan_waypoints.append((91.07747650146484,52.119384765625))
bunderan_waypoints.append((92.01168823242188,52.686004638671875))
bunderan_waypoints.append((92.93928527832031,53.24860763549805))
bunderan_waypoints.append((93.92679595947266,53.76810073852539))
bunderan_waypoints.append((94.93669891357422,54.20802307128906))
bunderan_waypoints.append((95.95890045166016,54.6533088684082))
bunderan_waypoints.append((96.95877838134766,55.0888671875))
bunderan_waypoints.append((97.95977020263672,55.54136657714844))
bunderan_waypoints.append((98.95046997070312,55.99018478393555))
bunderan_waypoints.append((99.97605895996094,56.42598342895508))
bunderan_waypoints.append((100.99517059326172,56.838623046875))
bunderan_waypoints.append((102.07270050048828,57.15061950683594))
bunderan_waypoints.append((103.16251373291016,57.304012298583984))
bunderan_waypoints.append((104.251708984375,57.330848693847656))

bunderan_waypoints.append((105.251708984375,57.330848693847656))
bunderan_waypoints.append((106.251708984375,57.330848693847656))
bunderan_waypoints.append((107.251708984375,57.330848693847656))
bunderan_waypoints.append((108.251708984375,57.330848693847656))
bunderan_waypoints.append((109.251708984375,57.330848693847656))

def main():
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()

    camera = world.get_spectator()
    camera.set_transform(carla.Transform(carla.Location(x=85.035,y=58.13,z=65.596),carla.Rotation(pitch=-89.0,yaw=-180.0,roll=0.0)))

    for waypoint in bunderan_waypoints:
        world.debug.draw_box(carla.BoundingBox(carla.Location(x=waypoint[0],y=waypoint[1],z=0),carla.Vector3D(0.1,0.1,0.2)),carla.Rotation(), 0.25, carla.Color(255,0,0,0),60.0)

    with open('trajectory.txt') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:
                print(f'\t{row[0]} works in the {row[1]}.')
                line_count += 1
        print(f'Processed {line_count} lines.')

    while(True):
        t = world.get_spectator().get_transform()
        camx = t.location.x
        camy = t.location.y
        cam_dir = t.rotation.yaw

        # indexing earliest point
        # distance_to_earliest_point = get_dist(camx,camy,bunderan_waypoints[0][0],bunderan_waypoints[0][1])
        # print(distance_to_earliest_point)
        # if distance_to_earliest_point<=5:
        #     bunderan_waypoints.pop(0)

        # get nearest waypoint
        jarak_min = 1000.0
        for i, waypoint in enumerate(bunderan_waypoints):
            jarak_temp = get_dist(camx,camy,waypoint[0],waypoint[1])
            if jarak_temp<=jarak_min:
                nearest_id = i
                jarak_min = jarak_temp

        # reward
        target_id = min(100,nearest_id+5)
        angle_to_target = angle_of_line(camx,camy,bunderan_waypoints[target_id][0],bunderan_waypoints[target_id][1])
        alpha = abs(angle_difference(cam_dir,angle_to_target))
        reward = 1/alpha
        reward = min(1,reward)
        print(round(reward,2),nearest_id,angle_to_target,alpha,target_id)

        # coordinate_str = "(x,y,z) = ({},{},{}) | (pitch,yaw,roll) = ({},{},{})".format(
        #     round(t.location.x,3), round(t.location.y,3), round(t.location.z,3),
        #     round(t.rotation.pitch,3), round(t.rotation.yaw,3), round(t.rotation.roll,3)
        #     )
        # print (coordinate_str)
        # time.sleep(_SLEEP_TIME_)

if __name__ == '__main__':
    main()


