from django.urls import path
from .views import login, info, logout


urlpatterns = [
    path('login', login),
    path('info', info),
    path('logout', logout),
]