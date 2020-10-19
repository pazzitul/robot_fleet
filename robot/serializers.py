"""
Model serializer for robot api
"""

from rest_framework import serializers
from robot.models import *


class RobotSerializer(serializers.ModelSerializer):

    class Meta:
        model = RobotModel
        fields = '__all__'


class RobotTypeSerializer(serializers.ModelSerializer):
    class Meta:
        model = RobotTypeModel
        fields = '__all__'
