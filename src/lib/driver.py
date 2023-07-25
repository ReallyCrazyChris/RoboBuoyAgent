import uasyncio as asyncio
from typecoersion import coerce
from machine import Pin

# initialize hardware (uncomment as needed)
_power = Pin(0, Pin.IN)
_onoff = Pin(14, Pin.OUT)


def startdriver(product):
    """ initialize and start the hardware driver as a daemon """

    # listen for product events  (uncomment as needed)
    product.on('onoff', lambda model, name,
               value: _onoff.value(coerce(value, 'integer')))

    # add a pollTask ot the event loop
    loop = asyncio.get_event_loop()
    loop.create_task(pollTask(product))


def stopdriver(product):
    """ stop and finalize the hardware driver """
    _onoff.value(0)


async def pollTask(product):
    """ asyncronously poll the hardware and commmit data to the product model """
    while 1:
        await asyncio.sleep(0.1)
        # comit _power input to model property power
        product.commit('power',  _power.value())
