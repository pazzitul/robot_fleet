from django.urls import path
from rest_framework.routers import DefaultRouter
from map.views import *

map_router = DefaultRouter()

map_router.register(prefix='points', viewset=PointAPI)
map_router.register(prefix='areas', viewset=AreaAPI)
map_router.register(prefix='', viewset=MapAPI)

urlpatterns = [
    path('get_destinations/', MapAPI.as_view({'get': 'get_destinations'})),
]

urlpatterns += map_router.urls