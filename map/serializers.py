"""
Model serializer for map api
"""

import yaml
from rest_framework import serializers

from map.models import *


class MapSerializer(serializers.ModelSerializer):

    config = serializers.SerializerMethodField(read_only=True)

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
        return config

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
