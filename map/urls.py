from django.urls import path
from rest_framework.routers import DefaultRouter
from map.views import *

map_router = DefaultRouter()
map_router.register(prefix='', viewset=MapAPI)

urlpatterns = [
]

urlpatterns += map_router.urls