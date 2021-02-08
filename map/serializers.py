"""
Model serializer for map api
"""

import yaml
from rest_framework import serializers

from map.models import *
from PIL import Image


class MapSerializer(serializers.ModelSerializer):

    config = serializers.SerializerMethodField(read_only=True)
    raw = serializers.JSONField(required=False)
    name = serializers.CharField(required=False)
    point_index = serializers.SerializerMethodField(read_only=True)
    area_index = serializers.SerializerMethodField(read_only=True)
    virtual_wall_index = serializers.SerializerMethodField(read_only=True)
    create_time = serializers.DateTimeField(format="%Y-%m-%d %H:%M", read_only=True)
    update_time = serializers.DateTimeField(format="%Y-%m-%d %H:%M", read_only=True)

    def get_config(self, map_obj):
        """
        fleet前端在加载png地图时需要获取map.yaml文件中的配置信息，该函数将读取yaml文件内容并添加到api的config字段中
        :param map_obj: 给定id的map数据对象
        :return:
        """
        file_path = settings.MEDIA_ROOT + map_obj.name + '/map.yaml'
        f = open(file_path, 'r', encoding='utf-8')
        content = f.read()
        f.close()
        config = yaml.load(content)
        image_path = settings.MEDIA_URL + map_obj.name + '/map.png'
        config['image_path'] = image_path
        config['id'] = map_obj.id
        with Image.open(settings.MEDIA_ROOT + map_obj.name + '/map.png') as img:
            config['width'] = img.size[0]
            config['height'] = img.size[1]
            map_obj.width = config['width']
            map_obj.height = config['height']
            map_obj.resolution = config['resolution']
            map_obj.origin = {
                'x': config['origin'][0],
                'y': config['origin'][1]
            }
            map_obj.save()
        return config

    def get_point_index(self, map_obj):
        return PointModel.objects.filter(map=map_obj).count() + 1

    def get_area_index(self, map_obj):
        return AreaModel.objects.filter(map=map_obj).count() + 1

    def get_virtual_wall_index(self, map_obj):
        return VirtualWallModel.objects.filter(map=map_obj).count() + 1

    class Meta:
        model = MapModel
        fields = '__all__'


class PointSerializer(serializers.ModelSerializer):

    class Meta:
        model = PointModel
        fields = '__all__'


class PointTypeSerializer(serializers.ModelSerializer):
    class Meta:
        model = PointTypeModel
        fields = '__all__'


class AreaSerializer(serializers.ModelSerializer):
    class Meta:
        model = AreaModel
        fields = '__all__'


class AreaTypeSerializer(serializers.ModelSerializer):
    class Meta:
        model = AreaTypeModel
        fields = '__all__'
