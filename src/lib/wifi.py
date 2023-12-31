import network 
from config import ssid, passwd
# join a wifi access point
def joinwifi():
    station = network.WLAN(network.STA_IF) # initiate a station mode

    if not station.isconnected():
            print('connecting to network:', ssid)
            station.active(True)
            station.connect(ssid, passwd)

            while not station.isconnected():
                pass

    # deactivating access point mode
    ap = network.WLAN(network.AP_IF)
    ap.active(False)

    ip = station.ifconfig()[0]
    print('connected as:', ip)

    return ip
