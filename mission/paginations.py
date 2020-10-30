from rest_framework.pagination import PageNumberPagination
from rest_framework.response import Response
from collections import OrderedDict


class MissionPagination(PageNumberPagination):
    page_size = 10
    page_size_query_param = 'pageSize'
    page_query_param = 'current'
    max_page_size = 100

    def get_paginated_response(self, data):
        return Response(OrderedDict([
            # ('next', self.get_next_link()),
            ('current', self.page.number),
            # ('previous', self.get_previous_link()),
            ('total', self.page.paginator.count),
            ('pageSize', self.page_size),
            ('data', data)
        ]))