from rest_framework.viewsets import ModelViewSet
from map.models import *


class MapAPI(ModelViewSet):
    queryset = MapModel


class PointAPI(ModelViewSet):
    queryset = PointModel


class AreaAPI(ModelViewSet):
    queryset = AreaModel


class PointTypeAPI(ModelViewSet):
    queryset = PointTypeModel


class AreaTypeAPI(ModelViewSet):
    queryset = AreaTypeModel
