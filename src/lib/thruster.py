import time
from machine import PWM, Pin
from math import radians
from lib.server import Server

server = Server()

class Thruster():
    """ Provides Dual Thruster Style Motor Control """

    def __init__(self):
    
        self.active = False
        self.surge = 0 # cm/s
        self.steer = 0 # deg/s

        self.vmin = 0 # cm/s
        self.vmax = 100 # cm/s
        self.gain = 1
        self.minpwm = 40
        self.maxpwm = 100 
        
        self.motorLeft = PWM(Pin(16))
        self.motorRight = PWM(Pin(17))
        self.motorLeft.freq(50)
        self.motorRight.freq(50)

        # register handlers
        server.addListener('t',self.sendstate)
        server.addListener('t/arm',self.arm)
        server.addListener('t/active',self.setactive)
        server.addListener('t/surge',self.setsurge)
        server.addListener('t/steer',self.setsteer)
        server.addListener('t/stop',self.stop)
        server.addListener('t/vmin', self.setvmin)
        server.addListener('t/vmax', self.setvmax)
        server.addListener('t/gain', self.setgain)
        server.addListener('t/minpwm',self.setminpwm)
    
        server.addListener('t/save',self.save)
        server.addListener('t/load',self.load)
  


    def sendstate(self,_):
        
        response = {
            "active":self.active, 
            "surge":self.surge, 
            "steer":self.steer, 
            "vmin":self.vmin, 
            "vmax":self.vmax, 
            "gain":self.gain, 
            "minpwm":self.minpwm, 
            "maxpwm":self.maxpwm, 
        }

        print('sending response',response)
        server.send('t',response)

    def setactive(self, active):
        print('active',active)
        self.active = bool(active)   
        self.drive(0,0)


    def setsurge(self, surge):
        self.surge = int(surge) 
        print('surge',self.surge)
        self.drive(self.steer,self.surge)

    def setsteer(self, steer):
        self.steer = int(steer)     
        print('steer',self.steer)
        self.drive(self.steer,self.surge) 

    def setvmin(self, vmin):
        print('vmin (cm/s)',vmin)
        self.vmin = int(vmin)

    def setvmax(self, vmax):
        print('vmax (cm/s)',vmax)
        self.vmax = int(vmax)
        self.drive(self.steer,self.surge)  


    def setgain(self, gain):
        print('gain',gain)
        self.gain = int(gain)
        self.drive(self.steer,self.surge)

        
    def setminpwm(self, minpwm):
        print('minpwm',minpwm)
        self.minpwm = int(minpwm)  
        self.surge = 1
        self.steer = 0
        self.drive(self.steer,self.surge)
    

    def drive(self, steer, surge):
        """
        steer in degrees -180..180
        surge in meters per second
        """
        self.surge = surge or self.surge
        self.steer = steer or self.steer
        
        vl = (2*surge + radians(steer)*self.gain) / 2
        vr = (2*surge - radians(steer)*self.gain) / 2

        # clamp max and min motor speeds  
        vl = min(self.vmax,vl)
        vl = max(self.vmin,vl)
        vr = min(self.vmax,vr)
        vr = max(self.vmin,vr)

        print('speed (cm/s)',vl,vr) 

        if self.active:

            pwm_left = (vl - self.vmin) * (self.maxpwm - self.minpwm) / (self.vmax - self.vmin) + self.minpwm
            pwm_left = int(pwm_left)
            self.motorLeft.duty(pwm_left)
    
            pwm_right = (vr - self.vmin) * (self.maxpwm - self.minpwm) / (self.vmax - self.vmin) + self.minpwm
            pwm_right = int(pwm_right)
            self.motorRight.duty(pwm_right)

        else:
            self.stop()
          
    def stop(self):
        """
        stops both motors
        """
        print('stopmotors')
        self.motorLeft.duty(0)
        self.motorRight.duty(0)

    def arm(self):
        """
        arms both motors
        """
        print('arm motors')
        self.motorLeft.duty(40)
        self.motorRight.duty(40)
        time.sleep(6)
        self.motorLeft.duty(115)
        self.motorRight.duty(115)
        time.sleep(6)
        self.motorLeft.duty(0)
        self.motorRight.duty(0)
        print('arm motors complete')

    def save(self, _ ):
        """write thruster to flash"""
        import json
        with open('thruster.json', 'w') as file:
            json.dump(self.toDict(), file)

    def load(self, _ ):
        """load thruster from flash"""
        import json 
        try:
            with open('thruster.json', 'r') as file:
                self.toObject( json.load(file) )
        except Exception :
            pass

    

    
        
        

