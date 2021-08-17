import settings
from sources import operating_system, STOP
import glob
import os
import sys
try:
    sys.path.append(glob.glob(settings.CARLA_PATH + f'/PythonAPI/carla/dist/carla-*{sys.version_info.major}.{sys.version_info.minor}-{"win-amd64" if os.name == "nt" else "linux-x86_64"}.egg')[0])
except IndexError:
    pass
import carla
import time
import random
import numpy as np
import math
from dataclasses import dataclass
import psutil
import subprocess
from queue import Queue

#custom
from carla import ColorConverter as cc
import csv


@dataclass
class ACTIONS:
    forward = 0
    left = 1
    right = 2
    forward_left = 3
    forward_right = 4
    brake = 5
    brake_left = 6
    brake_right = 7
    no_action = 8

ACTION_CONTROL = {
    # 0: [1, 0, 0],
    0: [0.75, 0, 0],
    1: [0, 0, -1],
    2: [0, 0, 1],
    # 3: [1, 0, -1],
    # 4: [1, 0, 1],
    3: [0.75, 0, -1],
    4: [0.75, 0, 1],
    5: [0, 1, 0],
    6: [0, 1, -1],
    7: [0, 1, 1],
    8: None,
}

ACTIONS_NAMES = {
    0: 'forward',
    1: 'left',
    2: 'right',
    3: 'forward_left',
    4: 'forward_right',
    5: 'brake',
    6: 'brake_left',
    7: 'brake_right',
    8: 'no_action',
}

# custom func start ========================
def angle_of_line(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2-y1, x2-x1))

def angle_difference(sourceA, targetA):
    a = targetA - sourceA
    a = (a + 180) % 360 - 180
    return a

def get_dist(x1,y1,x2,y2):
    return math.hypot(x2 - x1, y2 - y1)
# custom func end   ========================

# custom var start  =========================
on_sidewalk = False
# custom var end    =========================

# Carla environment
class CarlaEnv:
    # How much steering to apply
    STEER_AMT = 1.0

    # Image dimensions (observation space)
    im_width = settings.IMG_WIDTH
    im_height = settings.IMG_HEIGHT

    # Action space size
    action_space_size = len(settings.ACTIONS)

    # waypoints
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


    def __init__(self, carla_instance, seconds_per_episode=None, playing=False):
        # custom start =========================
        # variables tracking
        # self.speed_list=[]
        # custom end   =========================

        # Set a client and timeouts
        self.client = carla.Client(*settings.CARLA_HOSTS[carla_instance][:2])
        self.client.set_timeout(2.0)

        # Get currently running world
        self.world = self.client.get_world()

        # Get list of actor blueprints
        blueprint_library = self.world.get_blueprint_library()

        # Get Tesla model 3 blueprint
        self.model_3 = blueprint_library.filter('model3')[0]

        # Sensors and helper lists
        self.collision_hist = []
        self.actor_list = []
        self.front_camera = None
        self.preview_camera = None

        # Used to determine if Carla simulator is still working as it crashes quite often
        self.last_cam_update = time.time()

        # Updated by agents for statistics
        self.seconds_per_episode = seconds_per_episode

        # A flag indicating that we are just going to play
        self.playing = playing

        # Used with additional preview feature
        self.preview_camera_enabled = False

        # Sets actually configured actions
        self.actions = [getattr(ACTIONS, action) for action in settings.ACTIONS]

    # Resets environment for new episode
    def reset(self):
        # custom start ======================
        # spawn_point_list
        # map_list_custom = [
        #     self.world.get_map().get_spawn_points()[167],
        #     self.world.get_map().get_spawn_points()[181],
        #     carla.Transform(carla.Location(x=19.6,y=-11.7,z=1.8431),carla.Rotation(pitch=0, yaw=-133, roll=0)),
        #     carla.Transform(carla.Location(x=-13.5,y=-19.3,z=1.8431),carla.Rotation(pitch=0, yaw=136, roll=0))
        # ]
        map_list_custom = [
            carla.Transform(carla.Location(x=103.8,y=64.2,z=1.8431),carla.Rotation(pitch=0, yaw=-179, roll=0)),
            # carla.Transform(carla.Location(x=70.9,y=72.8,z=1.8431),carla.Rotation(pitch=0, yaw=-156, roll=0)),
            # carla.Transform(carla.Location(x=69.0,y=48.3,z=1.8431),carla.Rotation(pitch=0, yaw=-27, roll=0))
        ]
        # set camera location
        camera = self.world.get_spectator()
        # camera.set_transform(carla.Transform(carla.Location(x=0.0,y=0.0,z=70.0),carla.Rotation(pitch=-89.0,yaw=-180.0,roll=0.0)))
        # (x,y,z) = (85.035,58.13,65.596) | (pitch,yaw,roll) = (-89.0,-179.053,0.003) 
        camera.set_transform(carla.Transform(carla.Location(x=85.035,y=58.13,z=65.596),carla.Rotation(pitch=-89.0,yaw=-180.0,roll=0.0)))
        # variables tracking
        self.speed_list=[]
        self.episode_length_list=[]
        # self.angle_diff_list=[]
        # self.dist_diff_list=[]
        self.alpha_list=[]
        self.reward_list=[]

        self.finished=False
        self.stepcount=0
        self.action_list=[]
        self.prev_action=-1
        self.action_change_count=0
        self.car_x_list=[]
        self.car_y_list=[]
        # custom end   ======================

        # Car, sensors, etc. We create them every episode then destroy
        self.actor_list = []

        # When Carla breaks (stopps working) or spawn point is already occupied, spawning a car throws an exception
        # We allow it to try for 3 seconds then forgive (will cause episode restart and in case of Carla broke inform
        # main thread that Carla simulator needs a restart)
        spawn_start = time.time()
        while True:
            try:
                # Get random spot from a list from predefined spots and try to spawn a car there
                # custom
                # self.transform = random.choice(self.world.get_map().get_spawn_points())
                # self.transform = self.world.get_map().get_spawn_points()[186]
                # self.transform = self.world.get_map().get_spawn_points()[181]
                # self.transform = self.world.get_map().get_spawn_points()[167]
                self.transform = random.choice(map_list_custom)
                # for _ in range(200):
                #     print(self.transform)
                # self.transform = Transform(Location(x=-6.44617, y=-79.055, z=1.843), Rotation(pitch=0, yaw=92.0042, roll=0))
                # for i, loc in enumerate(self.world.get_map().get_spawn_points()):
                    # print(i,loc)
                self.vehicle = self.world.spawn_actor(self.model_3, self.transform)
                break
            except:
                time.sleep(0.01)

            # If that can't be done in 3 seconds - forgive (and allow main process to handle for this problem)
            if time.time() > spawn_start + 3:
                raise Exception('Can\'t spawn a car')

        # Append actor to a list of spawned actors, we need to remove them later
        self.actor_list.append(self.vehicle)

        #custom
        #draw custom spawn points
        # for waypoint in self.world.get_map().get_spawn_points():
        #     print(waypoint)
        #     self.world.debug.draw_string(waypoint.transform.location, 'o', draw_shadow=False,
        #                             color=carla.Color(r=255, g=255, b=255), life_time=0.2,
        #                             persistent_lines=True)
        # Get the blueprint for the camera
        # self.rgb_cam = self.world.get_blueprint_library().find('sensor.camera.rgb')
        if settings.USE_SEMANTIC_SEGMENTATION==True:
            self.rgb_cam = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        else:
            self.rgb_cam = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # Set sensor resolution and field of view
        self.rgb_cam.set_attribute('image_size_x', f'{self.im_width}')
        self.rgb_cam.set_attribute('image_size_y', f'{self.im_height}')
        self.rgb_cam.set_attribute('fov', '110')

        # Set camera sensor relative to a car
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))

        # Attach camera sensor to a car, so it will keep relative difference to it
        self.sensor = self.world.spawn_actor(self.rgb_cam, transform, attach_to=self.vehicle)

        # Register a callback called every time sensor sends a new data
        self.sensor.listen(self._process_img)

        # Add camera sensor to the list of actors
        self.actor_list.append(self.sensor)

        # Preview ("above the car") camera
        if self.preview_camera_enabled is not False:
            # Get the blueprint for the camera
            self.preview_cam = self.world.get_blueprint_library().find('sensor.camera.rgb')
            # Set sensor resolution and field of view
            self.preview_cam.set_attribute('image_size_x', f'{self.preview_camera_enabled[0]:0f}')
            self.preview_cam.set_attribute('image_size_y', f'{self.preview_camera_enabled[1]:0f}')
            self.preview_cam.set_attribute('fov', '110')

            # Set camera sensor relative to a car
            transform = carla.Transform(carla.Location(x=self.preview_camera_enabled[2], y=self.preview_camera_enabled[3], z=self.preview_camera_enabled[4]))

            # Attach camera sensor to a car, so it will keep relative difference to it
            self.preview_sensor = self.world.spawn_actor(self.preview_cam, transform, attach_to=self.vehicle)

            # Register a callback called every time sensor sends a new data
            self.preview_sensor.listen(self._process_preview_img)

            # Add camera sensor to the list of actors
            self.actor_list.append(self.preview_sensor)

        # Here's first workaround. If we do not apply any control it takes almost a second for car to start moving
        # after episode restart. That starts counting once we aplly control for a first time.
        # Workarounf here is to apply both throttle and brakes and disengage brakes once we are ready to start an episode.
        self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=1.0))

        # And here's another workaround - it takes a bit over 3 seconds for Carla simulator to start accepting any
        # control commands. Above time adds to this one, but by those 2 tricks we start driving right when we start an episode
        # But that adds about 4 seconds of a pause time between episodes.
        time.sleep(4)

        # Collision history is a list callback is going to append to (we brake simulation on a collision)
        self.collision_hist = []

        # Get the blueprint of the sensor
        colsensor = self.world.get_blueprint_library().find('sensor.other.collision')

        # Create the collision sensor and attach ot to a car
        self.colsensor = self.world.spawn_actor(colsensor, carla.Transform(), attach_to=self.vehicle)

        # Register a callback called every time sensor sends a new data
        self.colsensor.listen(self._collision_data)

        # Add the collision sensor to the list of actors
        self.actor_list.append(self.colsensor)

        # Almost ready to start an episide, reset camera update variable
        self.last_cam_update = time.time()

        # Wait for a camera to send first image (important at the beginning of first episode)
        while self.front_camera is None or (self.preview_camera_enabled is not False and self.preview_camera is None):
            time.sleep(0.01)

        # Disengage brakes
        self.vehicle.apply_control(carla.VehicleControl(brake=0.0))

        # Remember a time of episode start (used to measure duration and set a terminal state)
        self.episode_start = time.time()

        # Return first observation space - current image from the camera sensor
        return [self.front_camera, 0]

    # Collidion data callback handler
    def _collision_data(self, event):

        # What we collided with and what was the impulse
        collision_actor_id = event.other_actor.type_id
        collision_impulse = math.sqrt(event.normal_impulse.x ** 2 + event.normal_impulse.y ** 2 + event.normal_impulse.z ** 2)

        # Filter collisions
        for actor_id, impulse in settings.COLLISION_FILTER:
            if actor_id in collision_actor_id and (impulse == -1 or collision_impulse <= impulse):
                # custom start ======
                # sidewalk boleh
                # if collision_actor_id=='static.sidewalk':
                #     # self.collision_hist.append(event)
                #     global on_sidewalk
                #     on_sidewalk = True
                # custom end   ======
                return

        # Add collision
        self.collision_hist.append(event)

    # Camera sensor data callback handler
    def _process_img(self, image):
        #custom
        if settings.USE_SEMANTIC_SEGMENTATION==True:
            # road = [7,0,0]
            # white = [255,255,255]
            # image[np.all(image == road, axis=-1)] = white
            image.convert(cc.CityScapesPalette)
        # Get image, reshape and drop alpha channel
        image = np.array(image.raw_data)
        image = image.reshape((self.im_height, self.im_width, 4))
        image = image[:, :, :3]

        if settings.USE_SEMANTIC_SEGMENTATION==True:
            road = [128, 64, 128]
            roadLine = [50, 234, 157]
            white = [255,255,255]
            black = [0,0,0]
            roadMask = np.all(image == road,  axis=-1)
            roadLineMask = np.all(image == roadLine, axis=-1)
            image[roadMask | roadLineMask] = white
            image[~(roadMask | roadLineMask)] = black

        # Set as a current frame in environment
        self.front_camera = image

        # Measure frametime using a time of last camera update (displayed as Carla FPS)
        if self.playing:
            self.frametimes.append(time.time() - self.last_cam_update)
        else:
            self.frametimes.put_nowait(time.time() - self.last_cam_update)
        self.last_cam_update = time.time()

    # Preview camera sensor data callback handler
    def _process_preview_img(self, image):

        # If camera is disabled - do not process images
        if self.preview_camera_enabled is False:
            return

        # Get image, reshape and drop alpha channel
        image = np.array(image.raw_data)
        try:
            image = image.reshape((int(self.preview_camera_enabled[1]), int(self.preview_camera_enabled[0]), 4))
        except:
            return
        image = image[:, :, :3]

        # Set as a current frame in environment
        self.preview_camera = image

    # Steps environment
    def step(self, action):

        # Monitor if carla stopped sending images for longer than a second. If yes - it broke
        if time.time() > self.last_cam_update + 1:
            raise Exception('Missing updates from Carla')

        # Apply control to the vehicle based on an action
        if self.actions[action] != ACTIONS.no_action:
            self.vehicle.apply_control(carla.VehicleControl(throttle=ACTION_CONTROL[self.actions[action]][0], steer=ACTION_CONTROL[self.actions[action]][2]*self.STEER_AMT, brake=ACTION_CONTROL[self.actions[action]][1]))
            # custom for datas
            self.action_list.append(action)
            if self.prev_action!=action:
                self.action_change_count+=1
                self.prev_action = action
            # end custom for datas

        # Calculate speed in km/h from car's velocity (3D vector)
        v = self.vehicle.get_velocity()
        kmh = 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)

        done = False

        # # custom start ======================================
        # pusatx = -.5
        # pusaty = .5
        # dis = 21.5
        # car_trans = self.vehicle.get_transform()
        # car_yaw = car_trans.rotation.yaw
        # roundabout_to_car = angle_of_line(pusatx,pusaty,car_trans.location.x, car_trans.location.y)
        # angle_diff = angle_difference(roundabout_to_car, car_yaw)
        # dist = get_dist(pusatx,pusaty,car_trans.location.x,car_trans.location.y)
        # # if self.playing:
        # #     self.world.debug.draw_box(carla.BoundingBox(car_trans.location,carla.Vector3D(0.1,0.1,0.1)),car_trans.rotation, 0.5, carla.Color(255,0,0,0),5.0)
        # # custom end   ======================================


        # new custom start ==================================
        car_trans = self.vehicle.get_transform()
        car_yaw = car_trans.rotation.yaw
        # new custom end   ==================================

        # If car collided - end and episode and send back a penalty
        if len(self.collision_hist) != 0:
            # for _ in range(50):
            #     print(self.collision_hist[0])
            done = True
            reward = -1

        else:
            # get nearest waypoint
            jarak_min = 1000.0
            for i, waypoint in enumerate(self.bunderan_waypoints):
                jarak_temp = get_dist(car_trans.location.x,car_trans.location.y,waypoint[0],waypoint[1])
                if jarak_temp<=jarak_min:
                    nearest_id = i
                    jarak_min = jarak_temp
            # revisi
            if (not self.playing):
                if nearest_id==95:
                    done = True
            else:
                if nearest_id==85:
                    done = True
                    self.finished = True

            # reward
            target_id = min(100,nearest_id+5)
            # self.world.debug.draw_box(carla.BoundingBox(carla.Location(x=self.bunderan_waypoints[target_id][0],y=self.bunderan_waypoints[target_id][1],z=0),carla.Vector3D(0.1,0.1,0.2)),carla.Rotation(), 0.05, carla.Color(255,0,0,0),1.0)
            angle_to_target = angle_of_line(car_trans.location.x,car_trans.location.y,self.bunderan_waypoints[target_id][0],self.bunderan_waypoints[target_id][1])
            alpha = abs(angle_difference(car_yaw,angle_to_target))
            reward = 1/alpha
            reward = min(1,reward)
            # print(nearest_id,angle_to_target,alpha)

            # # reward 1 arah start ===================================
            # angle_diff_abs = (angle_diff) # range 45 sampe 135
            # angle_diff_abs += 90 # range -45 sampe 45
            # angle_diff_abs = abs(angle_diff_abs) # range 45 - 0 - 45
            # if angle_diff_abs>180:
            #     angle_diff_abs = abs(-180 + (angle_diff_abs-180))
            # # 180-270
            # if angle_diff_abs!=0:
            #     reward = 1/angle_diff_abs
            # reward = min(1,reward)
            # # reward 1 arah end   ===================================

            # # reward 2 jarak start ==================================
            # dist_proc = abs(dist-dis)
            # reward2 = 1/(dist_proc*10)
            # reward2 = min(1,reward2)
            # # reward 2 jarak end   ==================================

            # # Punishment 1 jarak start ==============================
            # punishment = 0
            # if dist > 30:
            #     punishment = .5
            # # Punishment 1 jarak end   ==============================

            # # reward total start ====================================
            # reward = reward+reward2-punishment
            # # reward total end   ====================================

            # # global on_sidewalk
            # # if on_sidewalk==True:
            # #     on_sidewalk=False
            # #     reward = -1000
            # #     for _ in range(50):
            # #         print("sidewalknew")

        # Reward NEW START ========================        
        # range -45 sampe -135
        # elif angle_diff < -45 and angle_diff > -135:
        #     angle_diff_abs = abs(angle_diff) # range 45 sampe 135
        #     angle_diff_abs -= 90 # range -45 sampe 45
        #     angle_diff_abs = abs(angle_diff_abs) # range 45 - 0 - 45
        #     angle_diff_abs = angle_diff_abs-45 # range 0 - -45 - 0
        #     angle_diff_abs = abs(angle_diff_abs) # range 0 - 45 - 0
        #     reward = angle_diff_abs/45 # range 0 - 1 - 0
        # # range 0 sampe -45 dan -135 sampe -179
        # elif ((angle_diff >= -45 and angle_diff < 0) or (angle_diff <= -135 and angle_diff >= -179)):
        #     reward = -.25
        # else:
        #     reward = -1

        # Reward NEW END   ========================

        # Reward OLD
        # elif settings.WEIGHT_REWARDS_WITH_SPEED == 'discrete':
        #     reward = settings.SPEED_MIN_REWARD if kmh < 50 else settings.SPEED_MAX_REWARD

        # elif settings.WEIGHT_REWARDS_WITH_SPEED == 'linear':
        #     reward = kmh * (settings.SPEED_MAX_REWARD - settings.SPEED_MIN_REWARD) / 100 + settings.SPEED_MIN_REWARD

        # elif settings.WEIGHT_REWARDS_WITH_SPEED == 'quadratic':
        #     reward = (kmh / 100) ** 1.3 * (settings.SPEED_MAX_REWARD - settings.SPEED_MIN_REWARD) + settings.SPEED_MIN_REWARD

        # If episode duration limit reached - send back a terminal state
        if not self.playing and self.episode_start + self.seconds_per_episode.value < time.time():
            done = True

        # Weights rewards (not for terminal state)
        if not self.playing and settings.WEIGHT_REWARDS_WITH_EPISODE_PROGRESS and not done:
            reward *= (time.time() - self.episode_start) / self.seconds_per_episode.value

        # custom start ====================
        # datas
        if (self.playing):
            if (done):
                # summary
                if not os.path.exists('./avg_data.csv'):
                    with open('avg_data.csv', mode='w', newline='') as avg_data_file:
                        avg_data_writer = csv.writer(avg_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        avg_data_writer.writerow(["finished","total_step","episode_length","total_speed","avg_speed",
                                                    "total_alpha","avg_alpha","total_reward","avg_reward",
                                                    "total_action","action_change_count","action_forward",
                                                    "action_forward_left","action_forward_right"])
                with open('avg_data.csv', mode='a', newline='') as avg_data_file:
                    avg_data_writer = csv.writer(avg_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    avg_data_writer.writerow([
                        self.finished,
                        self.stepcount,
                        round((time.time() - self.episode_start),3),
                        round(sum(self.speed_list),3),
                        round(np.mean(self.speed_list),3),
                        round(sum(self.alpha_list),3),
                        round(np.mean(self.alpha_list),3),
                        # round(np.mean(self.dist_diff_list),3),
                        round(sum(self.reward_list),3),
                        round(np.mean(self.reward_list),3),
                        len(self.action_list),
                        self.action_change_count,
                        self.action_list.count(0),
                        self.action_list.count(1),
                        self.action_list.count(2),
                        # self.action_list.count(3),
                        # self.action_list.count(4),
                        # self.action_list.count(5),
                        # self.action_list.count(6),
                        # self.action_list.count(7),
                        # self.action_list.count(8),
                        ])
                # trajectory
                if (self.finished):
                    if not os.path.exists('./trajectory.csv'):
                        with open('trajectory.csv', mode='w', newline='') as trajectory_data_file:
                            trajectory_data_writer = csv.writer(trajectory_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                            trajectory_data_writer.writerow(["x","y"])
                    with open('trajectory.csv', mode='a', newline='') as trajectory_data_file:
                        trajectory_data_writer = csv.writer(trajectory_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        for i in range(len(self.car_x_list)):
                            trajectory_data_writer.writerow([self.car_x_list[i],self.car_y_list[i]])
            else:
                # summary
                self.speed_list.append(kmh)
                self.alpha_list.append(alpha)
                # self.dist_diff_list.append(dist_proc)
                self.reward_list.append(reward)
                self.stepcount+=1
                # trajectory
                self.car_x_list.append(car_trans.location.x)
                self.car_y_list.append(car_trans.location.y)
        # custom end ======================

        return [self.front_camera, kmh], reward, done, None

    # Destroys all agents created from last .reset() call
    def destroy_agents(self):

        for actor in self.actor_list:

            # If it has a callback attached, remove it first
            if hasattr(actor, 'is_listening') and actor.is_listening:
                actor.stop()

            # If it's still alive - desstroy it
            if actor.is_alive:
                actor.destroy()

        self.actor_list = []


operating_system = operating_system()


# Returns binary
def get_binary():
    return 'CarlaUE4.exe' if operating_system == 'windows' else 'CarlaUE4.sh'


# Returns exec command
def get_exec_command():
    binary = get_binary()
    exec_command = binary if operating_system == 'windows' else ('./' + binary)

    return binary, exec_command


# tries to close, and if that does not work to kill all carla processes
def kill_processes():

    binary = get_binary()

    # Iterate processes and terminate carla ones
    for process in psutil.process_iter():
        if process.name().lower().startswith(binary.split('.')[0].lower()):
            try:
                process.terminate()
            except:
                pass

    # Check if any are still alive, create a list
    still_alive = []
    for process in psutil.process_iter():
        if process.name().lower().startswith(binary.split('.')[0].lower()):
            still_alive.append(process)

    # Kill process and wait until it's being killed
    if len(still_alive):
        for process in still_alive:
            try:
                process.kill()
            except:
                pass
        psutil.wait_procs(still_alive)


# Starts Carla simulator
def start(playing=False):
    # Kill Carla processes if there are any and start simulator
    if settings.CARLA_HOSTS_TYPE == 'local':
        print('Starting Carla...')
        kill_processes()
        for process_no in range(1 if playing else settings.CARLA_HOSTS_NO):
            subprocess.Popen(get_exec_command()[1] + f' -carla-rpc-port={settings.CARLA_HOSTS[process_no][1]}', cwd=settings.CARLA_PATH, shell=True)
            time.sleep(2)

    # Else just wait for it to be ready
    else:
        print('Waiting for Carla...')

    # Wait for Carla Simulator to be ready
    for process_no in range(1 if playing else settings.CARLA_HOSTS_NO):
        while True:
            try:
                client = carla.Client(*settings.CARLA_HOSTS[process_no][:2])
                map_name = client.get_world().get_map().name
                if len(settings.CARLA_HOSTS[process_no]) == 2 or not settings.CARLA_HOSTS[process_no][2]:
                    break
                if isinstance(settings.CARLA_HOSTS[process_no][2], int):
                    #custom
                    # print(client.get_available_maps())
                    map_choice = 'Town03'
                    # map_choice = random.choice([map.split('/')[-1] for map in client.get_available_maps()])
                else:
                    map_choice = settings.CARLA_HOSTS[process_no][2]
                if map_name != map_choice:
                    carla.Client(*settings.CARLA_HOSTS[process_no][:2]).load_world(map_choice)
                    while True:
                        try:
                            while carla.Client(*settings.CARLA_HOSTS[process_no][:2]).get_world().get_map().name != map_choice:
                                time.sleep(0.1)
                            break
                        except:
                            pass
                break
            except Exception as e:
                #print(str(e))
                time.sleep(0.1)


# Retarts Carla simulator
def restart(playing=False):
    # Kill Carla processes if there are any and start simulator
    if settings.CARLA_HOSTS_TYPE == 'local':
        for process_no in range(1 if playing else settings.CARLA_HOSTS_NO):
            subprocess.Popen(get_exec_command()[1] + f' -carla-rpc-port={settings.CARLA_HOSTS[process_no][1]}', cwd=settings.CARLA_PATH, shell=True)
            time.sleep(2)

    # Wait for Carla Simulator to be ready
    for process_no in range(1 if playing else settings.CARLA_HOSTS_NO):
        retries = 0
        while True:
            try:
                client = carla.Client(*settings.CARLA_HOSTS[process_no][:2])
                map_name = client.get_world().get_map().name
                if len(settings.CARLA_HOSTS[process_no]) == 2 or not settings.CARLA_HOSTS[process_no][2]:
                    break
                if isinstance(settings.CARLA_HOSTS[process_no][2], int):
                    #custom
                    # print(client.get_available_maps())
                    map_choice = 'Town03'
                    # map_choice = random.choice([map.split('/')[-1] for map in client.get_available_maps()])
                else:
                    map_choice = settings.CARLA_HOSTS[process_no][2]
                if map_name != map_choice:
                    carla.Client(*settings.CARLA_HOSTS[process_no][:2]).load_world(map_choice)
                    while True:
                        try:
                            while carla.Client(*settings.CARLA_HOSTS[process_no][:2]).get_world().get_map().name != map_choice:
                                time.sleep(0.1)
                                retries += 1
                                if retries >= 60:
                                    raise Exception('Couldn\'t change map [1]')
                            break
                        except Exception as e:
                            time.sleep(0.1)
                        retries += 1
                        if retries >= 60:
                            raise Exception('Couldn\'t change map [2]')

                break
            except Exception as e:
                #print(str(e))
                time.sleep(0.1)

            retries += 1
            if retries >= 60:
                break


# Parts of weather control code and npc car spawn code are copied from dynamic_weather.py and spawn_npc.py from examples, then modified
def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        self.altitude = 35.0 * (math.sin(self._t) + 1.0)


class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.puddles = 0.0
        self.wind = 0.0

    def tick(self, delta_seconds):
        delta = (1.3 if self._increasing else -1.3) * delta_seconds
        self._t = clamp(delta + self._t, -250.0, 100.0)
        self.clouds = clamp(self._t + 40.0, 0.0, 90.0)
        self.rain = clamp(self._t, 0.0, 80.0)
        delay = -10.0 if self._increasing else 90.0
        self.puddles = clamp(self._t + delay, 0.0, 75.0)
        self.wind = clamp(self._t - delay, 0.0, 80.0)
        if self._t == -250.0:
            self._increasing = True
        if self._t == 100.0:
            self._increasing = False


class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self.sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self.storm = Storm(weather.precipitation)

    def set_new_weather(self, weather):
        self.weather = weather

    def tick(self, delta_seconds):
        delta_seconds += random.uniform(-0.1, 0.1)
        self.sun.tick(delta_seconds)
        self.storm.tick(delta_seconds)
        self.weather.cloudyness = self.storm.clouds
        self.weather.precipitation = self.storm.rain
        self.weather.precipitation_deposits = self.storm.puddles
        self.weather.wind_intensity = self.storm.wind
        self.weather.sun_azimuth_angle = self.sun.azimuth
        self.weather.sun_altitude_angle = self.sun.altitude


# Carla settings states
@dataclass
class CARLA_SETTINGS_STATE:
    starting = 0
    working = 1
    restarting = 2
    finished = 3
    error = 4


# Carla settings state messages
CARLA_SETTINGS_STATE_MESSAGE = {
    0: 'STARTING',
    1: 'WORKING',
    2: 'RESTARING',
    3: 'FINISHED',
    4: 'ERROR',
}

# Carla settings class
class CarlaEnvSettings:

    def __init__(self, process_no, agent_pauses, stop=None, car_npcs=[0, 0], stats=[0., 0., 0., 0., 0., 0.]):

        # Speed factor changes how fast weather should change
        # self.speed_factor = 0.0
        self.speed_factor = 1.0

        # Process number (Carla instane to use)
        self.process_no = process_no

        # Weather and NPC variables
        self.weather = None
        self.spawned_car_npcs = {}

        # Set stats (for Tensorboard)
        self.stats = stats

        # Set externally to restarts settings
        self.restart = False

        # Controls number of NPCs and reset interval
        self.car_npcs = car_npcs

        # State for stats
        self.state = CARLA_SETTINGS_STATE.starting

        # External stop object (used to "know" when to exit
        self.stop = stop

        # We want to track NPC collisions so we can remove and spawn new ones
        # Collisions are really not rare when using built-in autopilot
        self.collisions = Queue()

        # Name of current world
        self.world_name = None

        # Controls world reloads
        self.reload_world_every = None if len(settings.CARLA_HOSTS[process_no]) == 2 or not settings.CARLA_HOSTS[process_no][2] or not isinstance(settings.CARLA_HOSTS[process_no][2], int) else (settings.CARLA_HOSTS[process_no][2] + random.uniform(-settings.CARLA_HOSTS[process_no][2]/10, settings.CARLA_HOSTS[process_no][2]/10))*60
        self.next_world_reload = None if self.reload_world_every is None else time.time() + self.reload_world_every

        # List of communications objects allowing Carla to pause agents (on changes like world change)
        self.agent_pauses = agent_pauses

    # Collect NPC collision data
    def _collision_data(self, collision):
        self.collisions.put(collision)

    # Destroys given car NPC
    def _destroy_car_npc(self, car_npc):

        # First check if NPC is still alive
        if car_npc in self.spawned_car_npcs:

            # Iterate all agents (currently car itself and collision sensor)
            for actor in self.spawned_car_npcs[car_npc]:

                # If actor has any callback attached - stop it
                if hasattr(actor, 'is_listening') and actor.is_listening:
                    actor.stop()

                # And if is still alive - destroy it
                if actor.is_alive:
                    actor.destroy()

            # Remove from car NPCs' list
            del self.spawned_car_npcs[car_npc]

    def clean_carnpcs(self):

        # If there were any NPC cars - remove attached callbacks from it's agents
        for car_npc in self.spawned_car_npcs.keys():
            for actor in self.spawned_car_npcs[car_npc]:
                if hasattr(actor, 'is_listening') and actor.is_listening:
                    actor.stop()

        # Reset NPC car list
        self.spawned_car_npcs = {}

    # Main method, being run in a thread
    def update_settings_in_loop(self):

        # Reset world name
        self.world_name = None

        # Reset weather object
        self.weather = None

        # Run infinitively
        while True:

            # Release agent pause locks, if there are any
            for agent_pause in self.agent_pauses:
                agent_pause.value = 0

            # Carla might break, make sure we can handle for that
            try:

                # If stop flag - exit
                if self.stop is not None and self.stop.value == STOP.stopping:
                    self.state = CARLA_SETTINGS_STATE.finished
                    return

                # If restart flag is being set - wait
                if self.restart:
                    self.state = CARLA_SETTINGS_STATE.restarting
                    time.sleep(0.1)
                    continue

                # Clean car npcs
                self.clean_carnpcs()

                # Connect to Carla, get worls and map
                self.client = carla.Client(*settings.CARLA_HOSTS[self.process_no][:2])
                self.client.set_timeout(2.0)
                self.world = self.client.get_world()
                self.map = self.world.get_map()
                self.world_name = self.map.name

                # Create weather object or update it if exists
                if self.weather is None:
                    self.weather = Weather(self.world.get_weather())
                else:
                    self.weather.set_new_weather(self.world.get_weather())

                # Get car blueprints and filter them
                self.car_blueprints = self.world.get_blueprint_library().filter('vehicle.*')
                self.car_blueprints = [x for x in self.car_blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                self.car_blueprints = [x for x in self.car_blueprints if not x.id.endswith('isetta')]
                self.car_blueprints = [x for x in self.car_blueprints if not x.id.endswith('carlacola')]

                # Get a list of all possible spawn points
                self.spawn_points = self.map.get_spawn_points()

                # Get collision sensor blueprint
                self.collision_sensor = self.world.get_blueprint_library().find('sensor.other.collision')

                # Used to know when to reset next NPC car
                car_despawn_tick = 0

                # Set state to working
                self.state = CARLA_SETTINGS_STATE.working

            # In case of error, report it, wait a second and try again
            except Exception as e:
                self.state = CARLA_SETTINGS_STATE.error
                time.sleep(1)
                continue

            # Steps all settings
            while True:

                # Used to measure sleep time at the loop end
                step_start = time.time()

                # If stop flag - exit
                if self.stop is not None and self.stop.value == STOP.stopping:
                    self.state = CARLA_SETTINGS_STATE.finished
                    return

                # Is restart flag is being set, break inner loop
                if self.restart:
                    break

                # Carla might break, make sure we can handle for that
                try:

                    # World reload
                    if self.next_world_reload and time.time() > self.next_world_reload:

                        # Set restart flag
                        self.state = CARLA_SETTINGS_STATE.restarting
                        # Clean car npcs
                        self.clean_carnpcs()

                        # Set pause lock flag
                        for agent_pause in self.agent_pauses:
                            agent_pause.value = 1

                        # Wait for agents to stop playing and acknowledge
                        for agent_pause in self.agent_pauses:
                            while agent_pause.value != 2:
                                time.sleep(0.1)

                        # Get random map and load it
                        #custom
                        map_choice = 'Town03'
                        # map_choice = random.choice(list({map.split('/')[-1] for map in self.client.get_available_maps()} - {self.client.get_world().get_map().name}))
                        self.client.load_world(map_choice)

                        # Wait for world to be fully loaded
                        retries = 0
                        while self.client.get_world().get_map().name != map_choice:
                            retries += 1
                            if retries >= 600:
                                raise Exception('Timeout when waiting for new map to be fully loaded')
                            time.sleep(0.1)

                        # Inform agents that they can start playing
                        for agent_pause in self.agent_pauses:
                            agent_pause.value = 3

                        # Wait for agents to start playing
                        for agent_pause in self.agent_pauses:
                            retries = 0
                            while agent_pause.value != 0:
                                retries += 1
                                if retries >= 600:
                                    break
                            time.sleep(0.1)

                        self.next_world_reload += self.reload_world_every

                        break

                    # Handle all registered collisions
                    while not self.collisions.empty():

                        # Gets first collision from the queue
                        collision = self.collisions.get()

                        # Gets car NPC's id and destroys it
                        car_npc = collision.actor.id
                        self._destroy_car_npc(car_npc)

                    # Count tick
                    car_despawn_tick += 1

                    # Carla autopilot might cause cars to stop in the middle of intersections blocking whole traffic
                    # On some intersections there might be only one car moving
                    # We want to check for cars stopped at intersections and remove them
                    # Without that most of the cars can be waiting around 2 intersections
                    for car_npc in self.spawned_car_npcs.copy():

                        # First check if car is moving
                        # It;s a simple check, not proper velocity calculation
                        velocity = self.spawned_car_npcs[car_npc][0].get_velocity()
                        simple_speed = velocity.x + velocity.y + velocity.z

                        # If car is moving, continue loop
                        if simple_speed > 0.1 or simple_speed < -0.1:
                            continue

                        # Next get current location of the car, then a waypoint then check if it's intersection
                        location = self.spawned_car_npcs[car_npc][0].get_location()
                        waypoint = self.map.get_waypoint(location)
                        if not waypoint.is_intersection:
                            continue

                        # Car is not moving, it's intersection - destroy a car
                        self._destroy_car_npc(car_npc)

                    # If we reached despawn tick, remove oldest NPC
                    # The reson we want to do that is to rotate cars aroubd the map
                    if car_despawn_tick >= self.car_npcs[1] and len(self.spawned_car_npcs):

                        # Get id of the first car on a list and destroy it
                        car_npc = list(self.spawned_car_npcs.keys())[0]
                        self._destroy_car_npc(car_npc)
                        car_despawn_tick = 0

                    # If there is less number of car NPCs then desired amount - spawn remaining ones
                    # but up to 10 at the time
                    if len(self.spawned_car_npcs) < self.car_npcs[0]:

                        # How many cars to spawn (up to 10)
                        cars_to_spawn = min(10, self.car_npcs[0] - len(self.spawned_car_npcs))

                        # Sometimes we can;t spawn a car
                        # It might be because spawn point is being occupied or because Carla broke
                        # We count errores and break on 5
                        retries = 0

                        # Iterate over number of cars to spawn
                        for _ in range(cars_to_spawn):

                            # Break if too many errors
                            if retries >= 5:
                                break

                            # Get random car blueprint and randomize color and enable autopilot
                            car_blueprint = random.choice(self.car_blueprints)
                            if car_blueprint.has_attribute('color'):
                                color = random.choice(car_blueprint.get_attribute('color').recommended_values)
                                car_blueprint.set_attribute('color', color)
                            car_blueprint.set_attribute('role_name', 'autopilot')

                            # Try to spawn a car
                            for _ in range(5):
                                try:
                                    # Get random spot from a list from predefined spots and try to spawn a car there
                                    spawn_point = random.choice(self.spawn_points)
                                    # spawn_point = self.world.get_map().get_spawn_points()[186]
                                    car_actor = self.world.spawn_actor(car_blueprint, spawn_point)
                                    car_actor.set_autopilot()
                                    break
                                except:
                                    retries += 1
                                    time.sleep(0.1)
                                    continue

                            # Create the collision sensor and attach it to the car
                            colsensor = self.world.spawn_actor(self.collision_sensor, carla.Transform(), attach_to=car_actor)

                            # Register a callback called every time sensor sends a new data
                            colsensor.listen(self._collision_data)

                            # Add the car and collision sensor to the list of car NPCs
                            self.spawned_car_npcs[car_actor.id] = [car_actor, colsensor]

                    # Tick a weather and set it in Carla
                    # self.weather.tick(self.speed_factor)
                    # self.world.set_weather(self.weather.weather)

                    # Set stats for tensorboard
                    self.stats[0] = len(self.spawned_car_npcs)
                    self.stats[1] = self.weather.sun.azimuth
                    self.stats[2] = self.weather.sun.altitude
                    self.stats[3] = self.weather.storm.clouds
                    self.stats[4] = self.weather.storm.wind
                    self.stats[5] = self.weather.storm.rain

                    # In case of state being some other one report that everything is working
                    self.state = CARLA_SETTINGS_STATE.working

                    # Calculate how long to sleep and sleep
                    sleep_time = self.speed_factor - time.time() + step_start
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                # In case of error, report it (reset flag set externally might break this loop only)
                except Exception as e:
                    #print(str(e))
                    self.state = CARLA_SETTINGS_STATE.error
