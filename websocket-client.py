import asyncio
from websockets.sync.client import connect
import json
import keyboard


with connect("ws://192.168.0.159:8765") as websocket:
    def send_key(event: keyboard.KeyboardEvent):
        websocket.send(json.dumps({"key": event.name, "event": event.event_type}))
                                 
    keyboard.hook(send_key)
    
    keyboard.wait("esc")
