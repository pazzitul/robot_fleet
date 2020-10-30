from rest_framework.viewsets import ModelViewSet

from mission.models import *
from mission.paginations import *
from mission.serializers import *


class MissionAPI(ModelViewSet):
    queryset = MissionModel.objects.all()
    serializer_class = MissionSerializer
    pagination_class = MissionPagination

    def list(self, request, *args, **kwargs):
        queryset = self.filter_queryset(self.get_queryset())
        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return Response(serializer.data)


class MissionTypeAPI(ModelViewSet):
    queryset = MissionTypeModel


class MissionGroupAPI(ModelViewSet):
    queryset = MissionGroupModel
