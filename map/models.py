import os
import shutil

from django.conf import settings
from django.db import models
from django.dispatch import receiver
from django_mysql.models import JSONField


def get_dir_name(instance, filename):
    """
    FIleField prefix dir
    """
    return '{0}/{1}'.format(instance.name, filename)


class MapModel(models.Model):
    """Model class of Map
    """

    name = models.CharField(max_length=64, unique=True)
    description = models.CharField(max_length=128, null=True)
    create_time = models.DateTimeField(auto_now_add=True)
    update_time = models.DateTimeField(auto_now=True)
    png = models.FileField(upload_to=get_dir_name, blank=True)
    yaml = models.FileField(upload_to=get_dir_name, blank=True)
    zip = models.FileField(upload_to=get_dir_name, null=True, blank=True)
    active = models.BooleanField(default=False)
    raw = JSONField(default=dict)

    class Meta:
        db_table = 'fleet_map'


class PointModel(models.Model):
    """Model class of Point
    """

    POINT_STATUS = (
        (0, 'UNAVAILABLE'),
        (1, 'AVAILABLE')
    )
    sn = models.IntegerField()
    name = models.CharField(max_length=64)
    type = models.ForeignKey('map.PointTypeModel', on_delete=models.CASCADE)
    position = JSONField(default='{"x": 0, "y": 0, "z": 0}')
    orientation = JSONField(default='{"x": 0, "y": 0, "z": 0}')
    status = models.IntegerField(choices=POINT_STATUS, default=1)
    map = models.ForeignKey('map.MapModel', on_delete=models.CASCADE)

    class Meta:
        db_table = 'fleet_point'


class PointTypeModel(models.Model):
    """Model class of Point Type
    """

    name = models.CharField(max_length=64)

    class Meta:
        db_table = 'fleet_point_type'


class PointActionModel(models.Model):
    """Model class of Point Action
    """

    name = models.CharField(max_length=64)
    type = models.ForeignKey('map.PointTypeModel', on_delete=models.CASCADE)

    class Meta:
        db_table = 'fleet_point_action'


class AreaModel(models.Model):
    """Model class of Area
    """

    AREA_STATUS = (
        (0, 'UNAVAILABLE'),
        (1, 'AVAILABLE')
    )
    sn = models.IntegerField()
    name = models.CharField(max_length=64)
    type = models.ForeignKey('map.AreaTypeModel', on_delete=models.CASCADE)
    vertices = JSONField(default=dict)
    status = models.IntegerField(choices=AREA_STATUS, default=1)
    map = models.ForeignKey('map.MapModel', on_delete=models.CASCADE)

    class Meta:
        db_table = 'fleet_area'


class AreaTypeModel(models.Model):
    """Model class of Area Type
    """

    name = models.CharField(max_length=64)

    class Meta:
        db_table = 'fleet_area_type'


class AreaActionModel(models.Model):
    """Model class of Area Action
    """

    name = models.CharField(max_length=64)
    type = models.ForeignKey('map.AreaTypeModel', on_delete=models.CASCADE)

    class Meta:
        db_table = 'fleet_area_action'


@receiver(models.signals.post_delete, sender=MapModel)
def auto_delete_file_on_delete(sender, instance, **kwargs):
    """
    该函数用于前端在调用map DELETE请求时。删除数据库记录的同时删除media目录下对应的文件夹
    :param sender:
    :param instance:
    :param kwargs:
    :return:
    """
    if instance.name:
        map_dir_path = settings.MEDIA_ROOT + instance.name
        if os.path.isdir(map_dir_path):
            shutil.rmtree(map_dir_path)