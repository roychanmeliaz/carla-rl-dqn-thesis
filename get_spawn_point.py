#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time
import settings

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


def main():
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()

    # get spawn points
    for i, spawn_point in enumerate(world.get_map().get_spawn_points()):
        print(spawn_point.location)
        # world.debug.draw_string(spawn_point.location, '.'+str(i), draw_shadow=False,
        # 						color=carla.Color(r=255, g=255, b=255), life_time=30.0,
        # 						persistent_lines=True)
        # # world.debug.draw_point( spawn_point.location, size=1, color=(255,0,0), life_time=10.0)
        world.debug.draw_box(carla.BoundingBox(spawn_point.location,carla.Vector3D(0.5,0.5,2)),spawn_point.rotation, 0.05, carla.Color(255,0,0,0),60.0)

    print("167", world.get_map().get_spawn_points()[167])
    print("181", world.get_map().get_spawn_points()[181])
    # 167 Transform(Location(x=-21.2525, y=11.105, z=1.8431), Rotation(pitch=0, yaw=61.8068, roll=0))
    # 181 Transform(Location(x=13.7238, y=18.8172, z=1.8431), Rotation(pitch=0, yaw=-38.797, roll=0))  

    while(True):
        t = world.get_spectator().get_transform()
        coordinate_str = "(x,y,z) = ({},{},{}) | (pitch,yaw,roll) = ({},{},{})".format(
            round(t.location.x,3), round(t.location.y,3), round(t.location.z,3),
            round(t.rotation.pitch,3), round(t.rotation.yaw,3), round(t.rotation.roll,3)
            )
        print (coordinate_str)
        time.sleep(_SLEEP_TIME_)

if __name__ == '__main__':
    main()


