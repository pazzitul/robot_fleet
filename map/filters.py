import django_filters

from map.models import PointModel, AreaModel, MapModel


class PointFilter(django_filters.rest_framework.FilterSet):
    class Meta:
        model = PointModel
        fields = [
            'map'
        ]


class AreaFilter(django_filters.rest_framework.FilterSet):
    class Meta:
        model = AreaModel
        fields = [
            'map',
            'type'
        ]


class MapFilter(django_filters.rest_framework.FilterSet):
    class Meta:
        model = MapModel
        fields = [
            'name',
            'active'
        ]
