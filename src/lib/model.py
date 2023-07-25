from lib.reactor import updatemodel
class Model():

    # Constructor
    def __init__(self):
        self.clazz = 'Model'
        self.type = 'model'
        self.id = False
        self.nodeid = False
        self.eventlisteners = {}
        self.wires = {}
        self.meta = {}
        self.props = {}
        self.ev = {}

    # Event
    # on - adds an event callback
    # @param n string event name
    # @param c fuction event callback
    def on(self, name, callback):
        if ((name in self.ev) == False):
            # create a queue for the destinaiton, push form and to
            self.ev[name] = []
        if ((name in self.ev) == True):
            self.ev[name].append(callback)  # push on the command and data

    # emit - emits an named event with arguments
    # @param n String event name
    # @param ... Arguments passed to the event callback
    def emit(self, name, *args):
        if (name in self.ev) == True:
            for callback in self.ev[name]:
                callback(*args)

    def commit(self, prop, value):
        updatemodel(self.nodeid,self.nodeid,'updatemodel',self.id, prop, value)
        
        
    # lifecycle start method
    def start(self):
        pass

    # lifecycle stop method
    def stop(self):
        pass

    # TODO is this for serialization ?
    def toDict(self):
        return {
            'clazz': self.clazz,
            'type': self.type,
            'id': self.id,
            'nodeid': self.nodeid,
            # 'eventlisteners' : self.eventlisteners,
            'wires': self.wires,
            'meta': self.meta,
            'props': self.props
        }
