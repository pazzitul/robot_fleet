from rest_framework.viewsets import ModelViewSet
from robot.models import *


class RobotAPI(ModelViewSet):
    queryset = RobotModel


class RobotTypeAPI(ModelViewSet):
    queryset = RobotTypeModel
