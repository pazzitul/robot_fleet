from django.contrib import admin
from django.urls import path, include
from django.conf.urls import url
from django.views.static import serve
from django.conf import settings
from .startup import startup


urlpatterns = [
    path('admin/', admin.site.urls),
    path('api/maps/', include('map.urls')),
    path('api/missions/', include('mission.urls')),
    path('api/robots/', include('robot.urls')),
    path('api/auth/', include('auth.urls')),
    path('api/strategies/', include('strategies.urls')),

    url(r'media/(?P<path>.*)', serve, {"document_root": settings.MEDIA_ROOT, 'show_indexes': True}),
]

startup()