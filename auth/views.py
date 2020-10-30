from django.shortcuts import render
from django.http import JsonResponse

# Create your views here.

roleObj = {
    'id': 'admin',
    'name': '管理员',
    'describe': '拥有所有权限',
    'status': 1,
    'creatorId': 'system',
    'createTime': 1497160610259,
    'deleted': 0,
    'permissions': [{
        'roleId': 'admin',
        'permissionId': 'dashboard',
        'permissionName': '仪表盘',
        'actions': '[{"action":"add","defaultCheck":false,"describe":"新增"},{"action":"query","defaultCheck":false,"describe":"查询"},{"action":"get","defaultCheck":false,"describe":"详情"},{"action":"update","defaultCheck":false,"describe":"修改"},{"action":"delete","defaultCheck":false,"describe":"删除"}]',
        'actionEntitySet': [{
            'action': 'add',
            'describe': '新增',
            'defaultCheck': False
        }, {
            'action': 'query',
            'describe': '查询',
            'defaultCheck': False
        }, {
            'action': 'get',
            'describe': '详情',
            'defaultCheck': False
        }, {
            'action': 'update',
            'describe': '修改',
            'defaultCheck': False
        }, {
            'action': 'delete',
            'describe': '删除',
            'defaultCheck': False
        }],
        'actionList': None,
        'dataAccess': None
    }, {
        'roleId': 'admin',
        'permissionId': 'exception',
        'permissionName': '异常页面权限',
        'actions': '[{"action":"add","defaultCheck":false,"describe":"新增"},{"action":"query","defaultCheck":false,"describe":"查询"},{"action":"get","defaultCheck":false,"describe":"详情"},{"action":"update","defaultCheck":false,"describe":"修改"},{"action":"delete","defaultCheck":false,"describe":"删除"}]',
        'actionEntitySet': [{
            'action': 'add',
            'describe': '新增',
            'defaultCheck': False
        }, {
            'action': 'query',
            'describe': '查询',
            'defaultCheck': False
        }, {
            'action': 'get',
            'describe': '详情',
            'defaultCheck': False
        }, {
            'action': 'update',
            'describe': '修改',
            'defaultCheck': False
        }, {
            'action': 'delete',
            'describe': '删除',
            'defaultCheck': False
        }],
        'actionList': None,
        'dataAccess': None
    }]
}

roleObj['permissions'].append({
    'roleId': 'admin',
    'permissionId': 'support',
    'permissionName': '超级模块',
    'actions': '[{"action":"add","defaultCheck":false,"describe":"新增"},{"action":"import","defaultCheck":false,"describe":"导入"},{"action":"get","defaultCheck":false,"describe":"详情"},{"action":"update","defaultCheck":false,"describe":"修改"},{"action":"delete","defaultCheck":false,"describe":"删除"},{"action":"export","defaultCheck":false,"describe":"导出"}]',
    'actionEntitySet': [{
        'action': 'add',
        'describe': '新增',
        'defaultCheck': False
    }, {
        'action': 'import',
        'describe': '导入',
        'defaultCheck': False
    }, {
        'action': 'get',
        'describe': '详情',
        'defaultCheck': False
    }, {
        'action': 'update',
        'describe': '修改',
        'defaultCheck': False
    }, {
        'action': 'delete',
        'describe': '删除',
        'defaultCheck': False
    }, {
        'action': 'export',
        'describe': '导出',
        'defaultCheck': False
    }],
    'actionList': None,
    'dataAccess': None
})


def login(request):
    return JsonResponse({'code': 20000, 'data': {'token': '123'}})


def info(request):
    return JsonResponse({'code': 20000, 'data': {'token': '123', 'role': roleObj}})


def logout(request):
    return JsonResponse({'code': 20000, 'data': {'token': '123'}})
