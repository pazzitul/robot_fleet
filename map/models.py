from django.db import models


class MapModel(models.Model):
    """
    Map Model
    """
    name = models.CharField(max_length=64)

    class Meta:
        db_table = 'fleet_map'
