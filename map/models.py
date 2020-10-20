from django.db import models


class MapModel(models.Model):
    """Model class of Map
    """

    name = models.CharField(max_length=64)
    description = models.CharField(max_length=128, null=True)
    create_time = models.DateTimeField(auto_now_add=True)
    update_time = models.DateTimeField(auto_now=True)
    active = models.BooleanField(default=False)
    raw = models.JSONField(default={})

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
    position = models.JSONField(default={'x': 0, 'y': 0, 'z': 0})
    orientation = models.JSONField(default={'x': 0, 'y': 0, 'z': 0, 'w': 1})
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
    vertices = models.JSONField(default={})
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
