from adatper.adapter import MASTER_ADAPTER
from map.models import *
from utils.coordinate import *
from . import planner_module


class Planner:
    @staticmethod
    def plan(robot, movements):
        adapter = MASTER_ADAPTER.get_robot_adapter(robot)
        if adapter:
            robot_position = adapter.get_position()
        else:
            robot_position = {
                'x': 0,
                'y': 0,
                'z': 0
            }
        map_model = MapModel.objects.filter(active=True).first()
        map_path = settings.MEDIA_ROOT + map_model.name
        global_path = []
        raw = []
        start = {}
        for i in range(0, len(movements)):
            if 'Point' in movements[i]['destination']:
                if i == 0:
                    start = robot_position
                    start_position = planner_module.position(start['x'], start['y'], start['z'], 0, 0, 0, 0)
                else:
                    start_position = planner_module.position(start['x'], start['y'], start['z'], 0, 0, 0, 0)
                end = PointModel.objects.filter(map__active=True,
                                                name=movements[i]['destination']
                                                ).first().position

                end_position = planner_module.position(end['x'], end['y'], end['z'], 0, 0, 0, 1)
                path = planner_module.PathVector()
                planner_module.point_to_point_path_planner(start_position, [end_position], map_path,
                                                           map_model.origin['x'],
                                                           map_model.origin['y'], map_model.resolution, 27, path)
                for step in path:
                    for point in step:
                        position = {
                            'x': point.x,
                            'y': point.y,
                            'z': 1
                        }
                        rotation = {
                            'x': point.p1,
                            'y': point.p2,
                            'z': point.p3,
                            'w': point.p4
                        }
                        global_path.append({
                                'position': position,
                                'rotation': rotation
                            })
                    raw.append({
                        'path': global_path,
                        'action': movements[i]['action']
                        })
                start = end

            if 'Area' in movements[i]['destination']:
                if start == {}:
                    start = robot_position
                    start_position = planner_module.position(start['x'], start['y'], start['z'], 0, 0, 0, 0)
                else:
                    start_position = planner_module.position(start['x'], start['y'], start['z'], 0, 0, 0, 0)
                m_vertices = AreaModel.objects.filter(map__active=True,
                                                      name=movements[i]['destination']
                                                      ).first().vertices
                areas = []
                for j in m_vertices:
                    pyp = planner_module.point(j['x'], j['y'])
                    areas.append(pyp)

                coverage_path = planner_module.PathVector()
                ratio = 0
                planner_module.coverage_path_planner(start_position,
                                                     areas,
                                                     settings.MEDIA_ROOT + map_model.name,
                                                     map_model.origin['x'],
                                                     map_model.origin['y'],
                                                     map_model.resolution,
                                                     15.0,
                                                     0.4,
                                                     planner_module.position(0, 0, 0, 0, 0, 0, 0),
                                                     planner_module.position(0, 0, 0, 0, 0, 0, 0),
                                                     coverage_path,
                                                     ratio
                                                     )

                for step in coverage_path:
                    for point in step:
                        position = {
                            'x': point.x,
                            'y': point.y,
                            'z': 1
                        }
                        rotation = {
                            'x': point.p1,
                            'y': point.p2,
                            'z': point.p3,
                            'w': point.p4
                        }
                        global_path.append({
                            'position': position,
                            'rotation': rotation
                        })
                        start = {
                            'x': point.x,
                            'y': point.y,
                            'z': point.z,
                            'p1': point.p1,
                            'p2': point.p2,
                            'p3': point.p3,
                            'p4': point.p4
                        }
                    raw.append({
                        'path': global_path,
                        'action': movements[i]['action']
                        })
        path_arr = []
        for k in global_path:
            position = map_to_canvas(k['position'])

            path_arr.append(position['x'])
            path_arr.append(position['y'])

        return path_arr, global_path, raw
