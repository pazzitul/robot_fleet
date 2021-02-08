from django.urls import path
from .planner.views import *


urlpatterns = [
    path('planning', PlannerAPI.as_view({'post': 'planning'}))
]
