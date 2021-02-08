from rest_framework.response import Response
from rest_framework.viewsets import ModelViewSet

from .planner import Planner


class PlannerAPI(ModelViewSet):
    def planning(self, request):
        print(request.data)
        robot = request.data.get('robot', None)
        movements = request.data.get('movements', None)

        path, _, _ = Planner.plan(robot, movements)

        return Response(data=path)
