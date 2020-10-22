from rest_framework.viewsets import ModelViewSet
from mission.models import *


class MissionAPI(ModelViewSet):
    queryset = MissionModel


class MissionTypeAPI(ModelViewSet):
    queryset = MissionTypeModel


class MissionGroupAPI(ModelViewSet):
    queryset = MissionGroupModel
