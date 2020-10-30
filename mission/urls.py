from django.urls import path
from rest_framework.routers import DefaultRouter
from mission.views import *


mission_router = DefaultRouter()
mission_router.register(prefix='', viewset=MissionAPI)

urlpatterns = [
]

urlpatterns += mission_router.urls