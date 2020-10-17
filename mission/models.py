from django.db import models


class MissionModel(models.Model):
    sn = models.CharField(max_length=64, unique=True)
    name = models.CharField(max_length=128, null=True)
    group = models.ForeignKey('mission.MissionGroupModel', on_delete=models.CASCADE, null=True)
    type = models.ForeignKey('mission.MissionTypeModel', on_delete=models.CASCADE, null=True)

    class Meta:
        db_table = 'fleet_mission'


class MissionGroupModel(models.Model):
    
    class Meta:
        db_table = 'fleet_mission_group'


class MissionTypeModel(models.Model):
    
    class Meta:
        db_table = 'fleet_mission_type'