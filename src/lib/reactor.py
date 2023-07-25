from config import nodekey
from lib.typecoersion import coerce
from lib.bencode import bdecode, bencode
from lib.store import Store

store = Store()  # get singleton of store
sendqueue = []
receivequeue = []
reactions = {}


def send(*packet):
    #print(packet)
    """sends commands to a node"""
    if nodekey == packet[1]:  # is packet for me
        receive(bdecode(bencode(packet)))
    else:
        sendqueue.append(packet)


def receive(packet):
    """recives and queue's commands to be reacted apon"""
    #print('receivequeue append', packet)
    receivequeue.append(packet)


def react():
    """ processed the commands in the receivequeue as a task in the asyncio loop"""
    #print('processing {} items in receivequeue'.format(len(receivequeue)))
    while len(receivequeue):

        packet = receivequeue.pop(0)
        
        if not packet[1] in [nodekey, 0]:
            return  # not for me and not a multicast
        if not packet[2] in reactions:
            return  # not a known reaction

        #print('react function', reactions[packet[2]])
        reactions[packet[2]](*packet)


def announce(fro, to, command):
    """ announces a product descritption"""
    send(nodekey, fro or 0, 'adddescription', store.models['1'].toDescription())


def updatemodel(fro, to, command, modelid, prop, value, salt=None):
    """ updates a property value, notifies shadow listeners and wire listeners """

    # TODO still needed ?
    if not to == nodekey:
        return send(nodekey, to, command, modelid, prop, value, salt)

    if (modelid in store.models) == False:
        return  # unknowm model
    model = store.models[modelid]
    if (prop in model.props) == False:
        return  # unknown prop

    # TODO reduce the size of the salt
    if salt == model.nodeid+modelid+prop:
        return  # circular race condition detected
    # detect future reace condition by setting salt
    salt = salt or model.nodeid+modelid+prop

    proptype = model.meta[prop]['type']

    # coerces the value to the property type
    coercedvalue = coerce(value, proptype)

    if model.props[prop] == coercedvalue:
        return  # no change to the value

    model.props[prop] = coercedvalue  # assigns the new value

    # emit a property value change event
    model.emit(prop, model, prop, model.props[prop])

    # propagate to shadow models
    for shadownodeid in store.shadowlisteners:
        send(nodekey, shadownodeid, 'updateshadowmodel',
             model.nodeid, model.id, prop, coercedvalue)

    if (prop in model.wires) == False:
        return  # any wire listeners for prop

    wirelisteners = model.wires[prop]
    # propagate the value to the wire listeners
    for listeneruri in wirelisteners:
        listenernodeid, listnermodlid, listenerprop = tuple(
            listeneruri.split('/'))
        send(nodekey, listenernodeid, 'updatemodel', listnermodlid, listenerprop, coercedvalue, salt)


def addwirelistener(fro, to, command, producer, consumer):
    """add wire listener, to be notified when a model property changes, updates/notifies shadow listeners"""

    pnodeid, pmodelid, pprop = tuple(producer.split('/'))
    cnodeid, cmodelid, cprop = tuple(consumer.split('/'))

    if (pmodelid in store.models) == False:
        return  # no such model

    model = store.models[pmodelid]

    if (pprop in model.props) == False:
        return  # no such key

    if (pprop in model.wires) == False:  # cretae a dict for the prop if there is none
        model.wires[pprop] = {}

    if (pprop in model.wires) == True:
        model.wires[pprop][consumer] = ''

    for shadowlistenerid in store.shadowlisteners:  # propagate to shadow listeners
        send(nodekey, shadowlistenerid,
             'shadowaddwirelistener', producer, consumer)

    #print('proagate {} to the wirelistener {}'.format(model.props[pprop],consumer))
    send(nodekey, cnodeid, 'updatemodel', cmodelid, cprop, model.props[pprop])

    # save the store state to non-volatile memory, so the wire state is remembered
    store.serialize()


def removewirelistener(fro, to, command, producer, consumer):
    """removes wire listener, updates/notifies shadow listeners"""
    pnodeid, pmodelid, pprop = tuple(producer.split('/'))
    # cnodeid,cmodelid,cprop = tuple(consumer.split('/'))

    if (pmodelid in store.models) == False:
        return  # no such model

    model = store.models[pmodelid]

    if (pprop in model.props) == False:
        return  # no such key

    if (pprop in model.wires) == False:
        return  # no such wire

    if (pprop in model.wires) == True:
        if (consumer in model.wires[pprop]) == True:
            del model.wires[pprop][consumer]

    for shadowlistenerid in store.shadowlisteners:  # propagate to shadow listeners
        send(nodekey, shadowlistenerid,
             'shadowremovewirelistener', producer, consumer)

    # save the store state to non-volatile memory, so the wire state is remembered
    store.serialize()


def addshadowlistener(fro, to, command, listenernodeid):
    """request to a remote node, so add self as a shadow listener"""
    # register the shadow listener
    store.shadowlisteners[listenernodeid] = True
    models = {}
    for modelid in store.models:
        models[modelid] = (store.models[modelid]).toDict()
    # notify the listener the current models
    send(nodekey, listenernodeid, 'addshadow', models)


def removeshadowlistener(fro, to, command, listenernodeid):
    """request to a remote node, so remove self as a shadow listener"""
    if listenernodeid in store.shadowlisteners:
        # unregister the shadow listener
        del store.shadowlisteners[listenernodeid]
        # notify the listener the current models
        send(nodekey, listenernodeid, 'removeshadow')


def addbrick(fro, to, command, packagename):
    """creats an instance of a brick"""
    #plugin = importlib.import_module('plugins.'+str(packagename), '.')
    plugin = __import__('plugins.'+str(packagename), fromlist=[None])
    instance = plugin.Brick()
    addmodel(instance)


def addmodel(model):
    """adds a Model instance to the store"""
    if (model.id in store.models) == True:
        return  # model exists

    if model.id == False:  # provide an id if the model has none
        store.pointer += 1
    model.id = str(store.pointer)  # model.id is a string
    model.nodeid = store.nodeid  # provide the model with a nodeid

    store.models[model.id] = model

    model.start()  # lifecycle start

    for shadowlistenerid in store.shadowlisteners:  # propagate to shadow listeners
        send(nodekey, shadowlistenerid, 'shadowaddmodel', model.toDict())


def removemodel(fro, to, command, modelid):
    print('removemodel', fro, to, command, modelid)
    if (modelid in store.models) == False:
        return  # model does not exists
    model = store.models[modelid]
    model.stop()

    for shadowlistenerid in store.shadowlisteners:  # propagate to shadow listeners
        send(nodekey, shadowlistenerid,'shadowremovemodel', model.id)

    del store.models[modelid]


""" from here on master node only """


def adddescription(fro, to, command, description):
    """add descrption, of a discovered Product"""
    store.discovered[fro] = description
    store.emit('adddescription', description, fro)


def addshadow(fro, to, command, shadow):
    """adds a shadow of a node, emits an event"""
    store.shadows[fro] = shadow          # register the shadowin shadows
    # shadow emits a shadow and shadows
    store.emit('addshadow', shadow, store.shadows)
 

def removeshadow(fro, to, command):
    """removes a shadow of a node, emits an event"""
    del store.shadows[fro]  # unregister the shadowin shadows
    # shadow emits a shadow and shadows
    store.emit('removeshadow', fro, store.shadows)


def updateshadowmodel(fro, to, command, nodeid, modelid, prop, value):
    """notifies shadow listeners of an model property change"""
    if (nodeid in store.shadows) == False:
        return  # unknown shadow
    shadow = store.shadows[nodeid]
    if (modelid in shadow) == False:
        return  # unknow shadowmodel
    shadowmodel = shadow[modelid]

    # update the value #TODO Check that the reference structure is correct
    shadowmodel['props'][prop] = value

    # shadow emits a key, value ,model
    store.emit('updateshadowmodel', prop, value, shadowmodel)


def shadowaddwirelistener(fro, to, command, producer, consumer):
    """shadow add wire listener, updates shadow state from orign model state"""
    nodeid, modelid, prop = tuple(producer.split('/'))

    if (nodeid in store.shadows) == False:
        return  # no such shadow

    shadow = store.shadows[nodeid]

    if (modelid in shadow) == False:
        return  # no such shadowmodel

    shadowmodel = shadow[modelid]

    if (prop in shadowmodel['props']) == False:
        return  # no such property

    # create a wire disctioniary if needed
    if (prop in shadowmodel['wires']) == False:
        shadowmodel['wires'][prop] = {}

    if (prop in shadowmodel['wires']) == True:
        # create a wire to the consumer
        shadowmodel['wires'][prop][consumer] = ''

    store.emit('shadowaddwire', producer, consumer)


def shadowremovewirelistener(fro, to, command, producer, consumer):
    """shadow remove wire listener, updates shadow state from orign model state"""
    nodeid, modelid, prop = tuple(producer.split('/'))

    if (nodeid in store.shadows) == False:
        return  # no such shadow

    

    shadow = store.shadows[nodeid]

    if (modelid in shadow) == False:
        return  # no such shadowmodel


    shadowmodel = shadow[modelid]

    if (prop in shadowmodel['wires']) == False:
        return  # no such proprty

    if (consumer in shadowmodel['wires'][prop]) == False:
        return

    del shadowmodel['wires'][prop][consumer]  # remove wire to the consumer

    store.emit('shadowremovewire', producer, consumer)


def shadowaddmodel(fro, to, command, model):
    """shadow add model, updates shadow state from orign model state"""
    nodeid = model['nodeid']
    modelid = model['id']

    if (nodeid in store.shadows) == False:
        return  # no such shadow

    shadow = store.shadows[nodeid]

    shadow[modelid] = model  # assign the shadowmodel

    store.emit('shadowaddmodel', shadow, model)


def shadowremovemodel(fro, to, command, modelid):
    """shadow remove model, updates shadow state from orign model state"""
    if (fro in store.shadows) == False:
        return  # no such shadow

    shadow = store.shadows[fro]

    if (modelid in shadow) == False:
        return  # no such shadowmodel

    del store.shadows[fro][modelid]  # delete the shadow model

    store.emit('shadowremovemodel', fro, modelid)


reactions = {
    'announce': announce,
    'updatemodel': updatemodel,
    'addwirelistener': addwirelistener,
    'removewirelistener': removewirelistener,
    'addshadowlistener': addshadowlistener,
    'removeshadowlistener': removeshadowlistener,
    'addbrick': addbrick,
    'removemodel': removemodel,

    'adddescription': adddescription,
    'addshadow': addshadow,
    'removeshadow': removeshadow,
    'shadowaddmodel': shadowaddmodel,
    'shadowremovemodel': shadowremovemodel,
    'updateshadowmodel': updateshadowmodel,
    'shadowaddwirelistener': shadowaddwirelistener,
    'shadowremovewirelistener': shadowremovewirelistener
}
