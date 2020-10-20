from django.contrib.auth.models import AbstractBaseUser, PermissionsMixin, BaseUserManager
from django.db import models


class UserManager(BaseUserManager):
    def _create_user(self, username, password, **kwargs):
        if not username:
            raise ValueError('Plz input username!')
        if not password:
            raise ValueError('Plz input password')

        user = self.model(username=username, **kwargs)
        user.set_password(password)
        user.save()

        return user

    def create_user(self, username, password, **kwargs):
        kwargs['is_superuser'] = False

        return self._create_user(username, password, **kwargs)

    def create_superuser(self, username, password, **kwargs):
        kwargs['is_superuser'] = True

        return self._create_user(username, password, **kwargs)


class User(AbstractBaseUser, PermissionsMixin):
    uid = models.UUIDField(primary_key=True)
    username = models.CharField(max_length=64, verbose_name="Username", unique=True)
    nickname = models.CharField(max_length=64, verbose_name="Nickname", null=True, blank=True)
    phone = models.CharField(max_length=11, verbose_name="Phone Number", null=True, blank=True)
    email = models.EmailField(verbose_name="Email", null=True, blank=True)
    is_active = models.BooleanField(default=True)
    date_joined = models.DateTimeField(auto_now_add=True)

    USERNAME_FIELD = 'username'
    REQUIRED_FIELDS = ['email']
    EMAIL_FIELD = 'email'

    objects = UserManager()

    def get_full_name(self):
        return self.username

    def get_short_name(self):
        return self.username
