from config import nodekey
from lib.reactor import send


def doaction(packet):
    """ acts out the command and data in the packet """

    if not packet[0] in actions:
        return  # not a known action
    action = packet.pop(0)
    #print('doaction',action)
    actions[action](*packet)

def discover(nodeid):
    """discovery of nodes onthe network by requesting other nodees to annouce-"""
    send(nodekey, nodeid or 0, 'announce')

def updatemodel(nodeid, modelid, prop, value, salt='0'):
    """set a property value of a local or remote model"""
    send(nodekey, nodeid, 'updatemodel', modelid, prop, value, salt)

def shadow(nodeid):
    """shadow another node"""
    send(nodekey, nodeid, 'addshadowlistener', nodekey)

def unshadow(nodeid):
    """unshadow another node"""
    send(nodekey, nodeid, 'removeshadowlistener', nodekey)

def wire(producer, consumer):
    """ request this agent to create a wire with another agent """
    nodeid = producer.split('/')[0]
    send(nodekey, nodeid, 'addwirelistener', producer, consumer)

def unwire(producer, consumer):
    """ request this agent to create a wire with another agent """
    nodeid = producer.split('/')[0]
    send(nodekey, nodeid, 'removewirelistener', producer, consumer)

def addbrick(packagename):
    """add a brick to a remote node"""
    send(nodekey, nodekey, 'addbrick', packagename)

def removemodel(nodeid,modelid):
    """remove a brick to a remote node"""
    send(nodekey, nodeid, 'removemodel', modelid)

"""actions that doactions may do"""
actions = {
    'discover': discover,
    'updatemodel': updatemodel,
    'shadow': shadow,
    'unshadow': unshadow,
    'wire': wire,
    'unwire': unwire,
    'addbrick': addbrick,
    'removemodel': removemodel
}
