import json

from rest_framework.viewsets import ModelViewSet

from adatper.adapter import MASTER_ADAPTER
from .paginations import *
from .serializers import *


class RobotAPI(ModelViewSet):
    queryset = RobotModel.objects.all()
    serializer_class = RobotSerializer

    # pagination_class = RobotPagination

    def list(self, request, *args, **kwargs):
        queryset = self.filter_queryset(self.get_queryset())
        page = self.paginate_queryset(queryset)
        if page is not None:
            serializer = self.get_serializer(page, many=True)
            return self.get_paginated_response(serializer.data)

        serializer = self.get_serializer(queryset, many=True)
        return Response(data=serializer.data)

    def get_registration_list(self, request):
        return Response(data=MASTER_ADAPTER.get_registration_list())

    def register(self, request):
        sn = request.data.get('sn', None)
        if sn is not None:
            MASTER_ADAPTER.register(sn)

        return Response(status=200, data=MASTER_ADAPTER.get_registration_list())

    def setting_map(self, request):
        data = request.data
        rep_msg = {
            'message_id': 1,
            'timestamp': 12,
            'message_type': 'map',
            'map_name': data['map_name'],
            'url': '/media/' + data['map_name'] + '/'
        }

        MASTER_ADAPTER.get_robot_adapter(data['sn']).map_switch(rep_msg)
        return Response(status=200, data='')


class RobotTypeAPI(ModelViewSet):
    queryset = RobotTypeModel
