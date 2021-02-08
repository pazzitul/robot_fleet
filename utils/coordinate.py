from map.models import MapModel
import math


def canvas_to_map(canvas_point, image_width, image_height, origin, resolution):
    image_x = canvas_point['x'] + image_width / 2
    image_y = canvas_point['y'] + image_height / 2

    return image_to_map({'x': image_x, 'y': image_y}, image_width, image_height, origin, resolution)


def image_to_map(image_point, image_width, image_height, origin, resolution):
    return {
        'x': image_point['x'] * resolution + origin['x'],
        'y': (image_height - image_point['y'] - 1) * resolution + origin['y'],
        'z': 0
    }


def map_to_canvas(map_point):
    map = MapModel.objects.filter(active=True).first()
    image_point = {'x': int((map_point['x'] - map.origin['x']) / map.resolution),
                    'y': int(map.height - (map_point['y'] - map.origin['y']) / map.resolution - 1), 'z': 0
                   }
    canvas_point = {
        'x': image_point['x'] - map.width / 2,
        'y': image_point['y'] - map.height / 2
    }
    return canvas_point


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians