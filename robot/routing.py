"""
Routing for robot websocket messages
"""
from django.urls import path
from .consumers import RobotConsumer

websocket_urlpatterns = [
    path('ws/robots/real_info/<str:sn>', RobotConsumer),
]