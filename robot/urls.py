from rest_framework.routers import DefaultRouter
from robot.views import *
from django.urls import path


robot_router = DefaultRouter()
robot_router.register(prefix='', viewset=RobotAPI)

urlpatterns = [
    path('get_registration_list', RobotAPI.as_view({'get': 'get_registration_list'})),
    path('register', RobotAPI.as_view({'post': 'register'})),
    path('setting/map', RobotAPI.as_view({'post': 'setting_map'}))
]

urlpatterns += robot_router.urls