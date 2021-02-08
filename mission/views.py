from uuid import uuid1

from rest_framework.viewsets import ModelViewSet

from mission.paginations import *
from mission.serializers import *
from strategies.planner.planner import Planner
from robot.models import RobotModel
from adatper.adapter import MASTER_ADAPTER


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

    def create(self, request, *args, **kwargs):
        robot = request.data.get('robot', None)
        movements = request.data.get('movements', None)

        _,path, raw = Planner.plan(robot, movements)
        msg = {
            'message_id': 1,
            'message_type': 'movement',
            'movements': raw
        }

        MissionModel.objects.create(global_path=path,
                                    robot=RobotModel.objects.get(sn=robot),
                                    raw=raw,
                                    sn=uuid1().__str__()[0:8])

        MASTER_ADAPTER.get_robot_adapter(robot).set_movements(msg)
        return Response()


class MissionTypeAPI(ModelViewSet):
    queryset = MissionTypeModel


class MissionGroupAPI(ModelViewSet):
    queryset = MissionGroupModel
