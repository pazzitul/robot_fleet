from channels.generic.websocket import AsyncWebsocketConsumer


class RobotConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        self.group_name = self.scope['url_route']['kwargs']['sn']
        message_type = self.scope['path'].split('/')[-2]
        await self.channel_layer.group_add(message_type + '_' + self.group_name, self.channel_name)
        await self.accept()

    async def disconnect(self, code):
        await self.channel_layer.group_discard(
            self.group_name,
            self.channel_name
        )

    async def real_info(self, event):
        message = event['message']
        await self.send(text_data=message)