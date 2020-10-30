from rest_framework.viewsets import ModelViewSet

from map.filters import *
from map.paginations import *
from map.serializers import *


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
        return Response(data={'code': 0, 'data': serializer.data})

    def destroy(self, request, *args, **kwargs):
        instance = self.get_object()
        self.perform_destroy(instance)
        queryset = self.get_queryset()
        return Response(self.get_serializer(queryset, many=True).data)


class PointAPI(ModelViewSet):
    queryset = PointModel


class AreaAPI(ModelViewSet):
    queryset = AreaModel


class PointTypeAPI(ModelViewSet):
    queryset = PointTypeModel


class AreaTypeAPI(ModelViewSet):
    queryset = AreaTypeModel
