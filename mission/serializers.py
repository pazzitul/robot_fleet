"""
Model serializer for mission api
"""

from rest_framework import serializers
from mission.models import *


class MissionSerializer(serializers.ModelSerializer):

    class Meta:
        model = MissionModel
        fields = '__all__'


class MissionGroupSerializer(serializers.ModelSerializer):
    class Meta:
        model = MissionGroupModel
        fields = '__all__'


class MissionTypeSerializer(serializers.ModelSerializer):
    class Meta:
        model = MissionTypeModel
        fields = '__all__'
