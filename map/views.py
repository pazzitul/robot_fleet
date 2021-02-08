from rest_framework.viewsets import ModelViewSet
from map.filters import *
from map.paginations import *
from map.serializers import *
from map.models import *
from utils.coordinate import canvas_to_map
from django.core.serializers.json import DjangoJSONEncoder
import json
from strategies.planner import planner_module


class MapAPI(ModelViewSet):
    queryset = MapModel.objects.all()
    serializer_class = MapSerializer
    filter_class = MapFilter

    # pagination_class = MapPagination

    def list(self, request, *args, **kwargs):
        queryset = self.filter_queryset(self.get_queryset())
        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return Response(data=serializer.data)

    def destroy(self, request, *args, **kwargs):
        instance = self.get_object()
        self.perform_destroy(instance)
        queryset = self.get_queryset()
        return Response(self.get_serializer(queryset, many=True).data)

    def create(self, request, *args, **kwargs):
        serializer = self.get_serializer(data=request.data)
        data = request.data

        try:
            print(data)
            name = data['name']
            yaml = data['yaml']
            png = data['png']
            if MapModel.objects.filter(name=name).count() > 0:
                return Response(status=300, data={'code': 11, 'data': '名为: {0} 的地图已存在.'.format(name)})
            print(name)
            if name == 'undefined':
                raise KeyError('name')

            serializer.is_valid(raise_exception=True)
            self.perform_create(serializer)
            return Response(data={'code': 20000, 'data': serializer.data})
        except KeyError as e:
            print(e)
            return Response(status=300, data={'code': 10, 'data': '缺少{0}'.format(e)})

    def update(self, request, *args, **kwargs):
        partial = kwargs.pop('partial', False)
        instance = self.get_object()
        serializer = self.get_serializer(instance, data=request.data, partial=partial)
        serializer.is_valid(raise_exception=True)
        self.perform_update(serializer)

        raw = request.data.get('raw', None)
        if raw is not None:
            for i in raw:
                if i['type'] == 1:
                    point = canvas_to_map({'x': i['x'], 'y': i['y']},
                                          instance.width,
                                          instance.height,
                                          instance.origin,
                                          instance.resolution)
                    PointModel.objects.update_or_create(name=str(i['name']), map_id=instance.id,
                                                        defaults={
                                                            'position': point
                                                        })
                elif i['type'] == 2:
                    c_vertices = [
                        {
                            'x': i['x'],
                            'y': i['y']
                        },
                        {
                            'x': i['x'] + i['width'],
                            'y': i['y']
                        },
                        {
                            'x': i['x'],
                            'y': i['y'] + i['height']
                        },
                        {
                            'x': i['x'] + i['width'],
                            'y': i['y'] + i['height']
                        }
                    ]
                    m_vertices = []
                    for p in c_vertices:
                        m_vertices.append(
                            canvas_to_map(p, instance.width, instance.height, instance.origin, instance.resolution))
                    AreaModel.objects.update_or_create(name=i['name'], map_id=instance.id,
                                                       defaults={
                                                           'vertices': m_vertices,
                                                       })
                elif i['type'] == 3:
                    c_vertices = []
                    for j in range(0, len(i['points']), 2):
                        c_vertices.append({
                            'x': i['points'][j],
                            'y': i['points'][j + 1]
                        })

                    m_vertices = []
                    for p in c_vertices:
                        m_vertices.append(
                            canvas_to_map(p, instance.width, instance.height, instance.origin, instance.resolution))
                    AreaModel.objects.update_or_create(name=i['name'], map_id=instance.id,
                                                       defaults={
                                                           'vertices': m_vertices,
                                                       })
                elif i['type'] == 4:
                    print(i)
                    c_vertices = []
                    for j in range(0, len(i['points']), 2):
                        c_vertices.append({
                            'x': i['points'][j],
                            'y': i['points'][j + 1]
                        })
                    m_vertices = []
                    for p in c_vertices:
                        m_vertices.append(
                            canvas_to_map(p, instance.width, instance.height, instance.origin, instance.resolution))
                    VirtualWallModel.objects.update_or_create(name=i['name'], map_id=instance.id,
                                                              defaults={
                                                                  'vertices': m_vertices,
                                                              })
        # 绘制虚拟墙
        walls = VirtualWallModel.objects.filter(map=instance)
        walls_list = []
        for i in walls:
            vertices_point_list = []
            for j in i.vertices:
                point = planner_module.point(j['x'], j['y'])
                vertices_point_list.append(point)
            walls_list.append(vertices_point_list)

        planner_module.set_visual_walls(settings.MEDIA_ROOT + instance.name, walls_list, instance.origin['x'], instance.origin['y'], instance.resolution)
        if getattr(instance, '_prefetched_objects_cache', None):
            # If 'prefetch_related' has been applied to a queryset, we need to
            # forcibly invalidate the prefetch cache on the instance.
            instance._prefetched_objects_cache = {}

        return Response(serializer.data)

    def get_destinations(self, request):
        map_id = request.query_params.get('id', None)
        if map_id:
            point_query_set = PointModel.objects.filter(map_id=map_id).values_list('name', flat=True)
            area_query_set = AreaModel.objects.filter(map_id=map_id).values_list('name', flat=True)

            pd = dict(zip(point_query_set, point_query_set))
            ad = dict(zip(area_query_set, area_query_set))
            ad.update(pd)

            return Response(data=ad)
        else:
            return Response(data={})


class PointAPI(ModelViewSet):
    queryset = PointModel.objects.all()
    serializer_class = PointSerializer

    def list(self, request, *args, **kwargs):
        queryset = self.filter_queryset(self.get_queryset())
        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return Response(data=serializer.data)


class AreaAPI(ModelViewSet):
    queryset = AreaModel.objects.all()


class PointTypeAPI(ModelViewSet):
    queryset = PointTypeModel


class AreaTypeAPI(ModelViewSet):
    queryset = AreaTypeModel
