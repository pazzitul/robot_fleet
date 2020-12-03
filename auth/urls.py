from django.urls import path
from .views import *


urlpatterns = [
    path('login', login),
    path('info', info),
    path('logout', logout),
    path('currentUser', current_user),
    path('login/account', account)
]