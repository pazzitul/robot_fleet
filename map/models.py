from django.db import models


class MapModel(models.Model):
    """@Model class of Map
    """
    name = models.CharField(max_length=64)

    class Meta:
        db_table = 'fleet_map'


class PointModel(models.Model):
    """@Model class of Point

    """

    class Meta:
        db_table = 'fleet_point'


class PointTypeModel(models.Model):
    """@Model class of Point Type
    """

    class Meta:
        db_table = 'fleet_point_type'

class AreaModel(models.Model):
    """@Model class of Area
    """

    class Meta:
        db_table = 'fleet_area'


class AreaModelType(models.Model):
    """@Model class of Area Type
    """

    class Meta:
        db_table = 'fleet_area_type'