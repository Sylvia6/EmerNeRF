from . import msg
from .genpy import Message
import inspect

message_register = {}


def register_classes():
    for name, obj in inspect.getmembers(msg):
        if inspect.isclass(obj):
            if issubclass(obj, Message) and obj != Message:
                message_register[obj._type] = obj

register_classes()