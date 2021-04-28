
import glob
import os
import sys

try:
    path = '/home/jaqq/YOLO/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg'
    sys.path.append(glob.glob(path % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print(sys.version_info.major, sys.version_info.minor)


import carla
import json
import math
import random
import time
import weakref

from abc import ABCMeta, abstractmethod
     
class CameraManager():
    actor = None
    cameras = {}
    camera_configs = None
    world = None
    
    def __init__(self, actor, camera_configs):

        self.actor = actor
        self.world = self.actor.get_world()
        self.camera_configs = camera_configs

    def init_cameras(self):
     
        for camera_name, camera_config in self.camera_configs.items():
            self.init_camera(camera_name, camera_config)
    
    def init_camera(self, camera_name, camera_config):

        cam_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')

        for attr_name, attr_value in camera_config['intrinsics'].items():
            cam_bp.set_attribute(attr_name, str(attr_value))
        
        location = (camera_config["x"], camera_config["y"], camera_config["z"])
        rotation = (camera_config["pitch"], camera_config["yaw"], camera_config["roll"])
        
        cam_location = carla.Location(*location)
        cam_rotation = carla.Rotation(*rotation)
        cam_transform = carla.Transform(cam_location, cam_rotation) 
        
        self.cameras[camera_name] = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.actor)
        self.cameras[camera_name].listen(lambda image: image.save_to_disk('./outputs/' + camera_name + '%.6d.jpg' % image.frame_number))

class RadarManager():

    actor = None
    radars = {}
    radar_configs = None
    world = None

    def __init__(self, actor, radar_configs):

        self.actor = actor
        self.world = self.actor.get_world()
        self.radar_configs = radar_configs

    def init_radars(self):

        for radar_name, radar_config in self.radar_configs.items():
            self.init_radar(radar_name, radar_config)

    def init_radar(self, radar_name, radar_config):

        rad_bp = self.world.get_blueprint_library().find('sensor.other.radar')

        for attr_name, attr_value in radar_config['intrinsics'].items():
            rad_bp.set_attribute(attr_name, str(attr_value))
        
        location = (radar_config["x"], radar_config["y"], radar_config["z"])
        rotation = (radar_config["pitch"], radar_config["yaw"], radar_config["roll"])

        rad_location = carla.Location(*location)
        rad_rotation = carla.Rotation(*rotation)
        rad_transform = carla.Transform(rad_location,rad_rotation)
        
        self.radars[radar_name] = self.world.spawn_actor(rad_bp, rad_transform, attach_to=self.actor)
        self.radars[radar_name].listen(lambda radar_data: self.rad_callback(radar_data))

    def rad_callback(self, radar_data):
        print("rad callback")
        velocity_range = 7.5 # m/s
        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.world.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))


class Tesla():
    bp = None
    actor = None
    world = None
    camera_configs = None
    CameraManager = None

    def __init__(self, world):

        self.world = world

        blueprint_library = self.world.get_blueprint_library()
        self.bp = random.choice(blueprint_library.filter('tesla'))

        self.camera_configs = {
            "front_camera": self.load_config(os.path.join("sensors", "camera", "front_camera.json")),
            "rear_camera": self.load_config(os.path.join("sensors", "camera", "rear_camera.json"))
        }

        self.radar_configs = {
            "front_radar": self.load_config(os.path.join("sensors", "radar", "default.json")),
        }

        self.init_car()
    
    def init_car(self):

        self.bp.set_attribute('color', "255,255,255")
        transform = self.world.get_spectator().get_transform()
        self.actor = self.world.spawn_actor(self.bp, transform)    
        self.cameraManager = CameraManager(self.actor, self.camera_configs)
        self.radarManager = RadarManager(self.actor, self.radar_configs)
        self.init_sensors()

    def init_sensors(self):
        self.cameraManager.init_cameras()
        self.radarManager.init_radars()

    def load_config(self, path):
        with open(path) as f:
            return json.load(f)

    def drive(self, method="autopilot"):

        if method == "autopilot":
            self.actor.set_autopilot(True)
        else:
            raise NotImplementedError("no such MMMMMMMMMMMMM")

def main():

    print("CHUCHU")

    client = carla.Client('localhost', 2000)
    client.set_timeout(20)

    world = client.get_world()

    tesla = Tesla(world)
    tesla.drive()

    while True:
        world_snapshot = world.wait_for_tick()

if __name__ == '__main__':

    main()
