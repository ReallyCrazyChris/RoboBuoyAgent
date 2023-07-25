
import uasyncio as asyncio
from config import ip
from lib.wifi import joinwifi
from lib.product import Product
from lib.reactor import addmodel, announce, react
from lib.udptransport import getsocket, receiveudp, sendudp

async def reactTask(sock):
    while 1:
        receiveudp(sock)
        react()
        sendudp(sock)
        await asyncio.sleep(0)

async def websockTask(websock):
    while 1:
        websock.serveonce()
        await asyncio.sleep(0)

async def heartbeatTask():
    while 1:
        await asyncio.sleep(10)
        announce('','','')

async def main_task():

    sock = getsocket(ip)
    asyncio.create_task( reactTask(sock) )
    asyncio.create_task( heartbeatTask() )

    while 1:
        await asyncio.sleep(60)
        print('still alive')

if __name__ == "__main__":

        print('WireUp Agent v0.1')
        product = Product()
        addmodel(product)
        asyncio.run( main_task() )

