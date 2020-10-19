"""
Model serializer for map api
"""

from rest_framework import serializers
from map.models import *


class MapSerializer(serializers.ModelSerializer):

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
