from django.db import models


class MissionModel(models.Model):
    """Model class of Mission
    """

    MISSION_STATUS = (
        (0, 'RAW'),
        (1, 'DISPATCHABLE'),
        (2, 'BEING_PROCESSED'),
        (3, 'FINISHED'),
        (4, 'FAILED'),
        (5, 'PAUSE'),
        (6, 'WITHDRAW')
    )
    sn = models.CharField(max_length=64, unique=True)
    name = models.CharField(max_length=128, null=True)
    group = models.ForeignKey('mission.MissionGroupModel', on_delete=models.CASCADE, null=True)
    type = models.ForeignKey('mission.MissionTypeModel', on_delete=models.CASCADE, null=True)
    status = models.IntegerField(choices=MISSION_STATUS, default=0)
    create_time = models.DateTimeField(auto_now_add=True)
    begin_time = models.DateTimeField()
    finish_time = models.DateTimeField()
    global_path = models.TextField(null=True)
    track_path = models.TextField(null=True)
    progress = models.IntegerField(null=True)

    class Meta:
        db_table = 'fleet_mission'


class MissionGroupModel(models.Model):
    """Model class of Mission Group
    """

    class Meta:
        db_table = 'fleet_mission_group'


class MissionTypeModel(models.Model):
    """Model class of Mission Type
    """

    class Meta:
        db_table = 'fleet_mission_type'
