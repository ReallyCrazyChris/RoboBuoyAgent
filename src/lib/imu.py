"""
Micropython driver for the I2C MPU9250 9-DOF Sensor
"""
import utime
from struct import pack, unpack
from math import atan2, degrees, sqrt, radians

class IMU(object):
    '''
    Class provides 9-DOF IMU sensor information for the MPU9250
    North-East-Down(NED) as a fixed, parent coordinate system

    TODO: Consider correcting the accel and gyro NED frame using the mpu9250 mounting matrix capabiilty
    TODO: Persist and Restore the constants
    '''

    def __init__( self, i2c ):

        #Networking
        self.i2c = i2c 

        #Constants
        self.accelbias = (0,0,1)
        self.gyrobias = (0,0,0)
        self.magbias = (20.03906, -23.30859, 17.7207, 48.9375, 54.10547, 36.19727, 0.9484222, 0.8578321, 1.282235)
        self.declination = 0
        self.tempoffset = 0
        self.tempsensitivity = 321

        #AccelInit
        self.accel = (0,0,0) 
        self.accelSSF = 16384
        self.accelfullScaleRange(fullScaleRange=0)

        #GyroInit
        self.gyroSSF = 131
        self.gyrofullScaleRange(fullScaleRange = 0) #250_deg/s
        self.gyroLowPassFilter(bandwidth = 6) #5Hz

        #MagInit
        self.initMag()

        #DeltaT
        self.startTime = None

        #Load saved calibration data
        self.load()

    #Accelerometer
    def accelfullScaleRange( self, fullScaleRange=0 ):
        '''
        Sets and reads the Accelerometers operating range and low pass filter frequencies
        fullScaleRange: 0,1,2,3 => +-2g,+-4g,+-8g,+-16g
        returns fullScaleRange
        ''' 
        if fullScaleRange != None and fullScaleRange in [0,1,2,3]:
            self.i2c.writeto_mem(0x69, 0x1C, pack('b',
            (self.i2c.readfrom_mem(0x69, 0x1C, 1)[0] & ~24) | fullScaleRange << 3
            ))

            # pick the accelerometer Sensitivity Scale Factor    
            self.accelSSF = 16834 #[16384,8192,4096,2048][fullScaleRange] 
    
        return (self.i2c.readfrom_mem(0x69, 0x1C, 1)[0] & 24) >> 3 

    def readAccel( self ):
        """
        return tuple of accelerations (x,y,z)
        North-East-Down(NED) as a fixed, parent coordinate system
        """

        y,x,z = unpack('>hhh',self.i2c.readfrom_mem(0x69, 0x3B, 6)) 

        x = x / self.accelSSF
        y = -1 * y / self.accelSSF
        z = -1 * z / self.accelSSF

        return x,y,z


    def meanAccel( self, samples=10, delay=10):
        """
        creates accelerometer averaged value sof a number of samles with a 
        sample interval
        
        """
        ox, oy, oz = 0.0, 0.0, 0.0
        n = float(samples)

        while samples:
            utime.sleep_ms(delay)
            gx, gy, gz = self.readAccel()
            ox += gx
            oy += gy
            oz += gz
            samples -= 1

        # mean accel values
        ox,oy,oz = ox / n, oy / n, oz / n

        return ox,oy,oz  

    def calibrateAccel( self, samples=10, delay=10 ):
        ''' Save the accel mean as the accel bias to the imu store'''
        self.accelbias = self.meanAccel( samples, delay )
        self.save()


    def readCalibractedAccel(self):
        ''' apply the calibrated accel bias to the raw accel values'''
        x,y,z = self.readAccel()
        xo, yo, zo = self.accelbias
        return x-xo, y-yo, z-zo, self.deltat()  

    #Gyro
    def gyrofullScaleRange(self, fullScaleRange=None ):
        """    
        Sets and reads the Gyro full scal operting range
        fullScaleRange: 0,1,2,3 => +-250, +-500, +-1000, +-2000 degrees/second  
        """
        if fullScaleRange != None and fullScaleRange in [0,1,2,3]:
            self.i2c.writeto_mem(0x69, 0x1B, pack('b',
            (self.i2c.readfrom_mem(0x69, 0x1B, 1)[0] & ~24) | fullScaleRange << 3
            ))

            # pick the gyro Sensitivity Scale Factor    
            self.gyroSSF = 131 #[131,65.5,32.8,16.4][fullScaleRange] 

        return (self.i2c.readfrom_mem(0x69, 0x1B, 1)[0] & 24) >> 3 

    def gyroLowPassFilter(self, bandwidth=None ):
        """    
        Sets and reads the Gyro operating range and low pass filter frequencies
        bandwidth: 0,1,2,3,4,5,6,7 => 250Hz, 184Hz, 92Hz, 41Hz, 20Hz, 10Hz, 5Hz, 3600Hz
        """
        if bandwidth and bandwidth in [0,1,2,3,4,5,6,7]:
            self.i2c.writeto_mem(0x69, 0x1A, pack('b',
            (self.i2c.readfrom_mem(0x69, 0x1A, 1)[0] & ~7 ) | bandwidth
            ))

        return self.i2c.readfrom_mem(0x69, 0x1A, 1)[0] & 7  

    def readGyro( self ):
        """
        return tuple of degrees per second (x,y,z)
        North-East-Down(NED) as a fixed, parent coordinate system
        """
        y,x,z = unpack('>hhh',self.i2c.readfrom_mem(0x69, 0x43, 6)) 
        x = x / self.gyroSSF
        y = y / self.gyroSSF
        z = z / self.gyroSSF
        z = -1 * z

        return x,y,z         

    def meanGyro( self, samples=10, delay=10 ):
        
        ox, oy, oz = 0.0, 0.0, 0.0
        n = float(samples)

        while samples:
            utime.sleep_ms(delay)
            gx, gy, gz = self.readGyro()
            ox += gx
            oy += gy
            oz += gz
            samples -= 1

        # mean gyro values
        ox,oy,oz = ox / n, oy / n, oz / n

        return ox,oy,oz  

    def calibrateGyro( self, samples=10, delay=10 ):
        ''' Saves the Gyro mean as the Gyro bias to the imu store'''
        self.gyrobias = self.meanGyro( samples, delay )
        self.save()


    def readCalibractedGyro(self):
        ''' apply the calibrated accel bias to the raw accel values'''
        x,y,z = self.readGyro()
        xo, yo, zo = self.gyrobias

        return x-xo, y-yo, z-zo, self.deltat()        

    #Magnetometer
    def initMag( self ):
        # Directly access the magnetomoeter via I2C BYPASS mode
        try:
            self.i2c.writeto_mem(0x69, 0x6B, b'\x80') #PWR_MGMT_1 = H_RESET # Rest the MPU6050 Magnetomoeter
            self.i2c.writeto_mem(0x69, 0x6A, b'\x00') #USER_CTRL_AD = I2C_MST = 0x00 disable i2c master
            self.i2c.writeto_mem(0x69, 0x37, b'\x02') #INT_PIN_CFG = BYPASS[1]
        except OSError as e:
            print('please check the MPU9250 I2C wiring ')

        # Read the Factory set Magntometer Sesetivity Adjustments
        self.i2c.writeto_mem(0x0C, 0x0A, b'\x1F') #CNTL1 Fuse ROM mode
        utime.sleep_ms(100) # Settle Time

        # Read factory calibrated sensitivity constants
        asax, asay, asaz = unpack('<bbb',self.i2c.readfrom_mem(0x0C, 0x10, 3)) 

        # Calculate the Magnetometer Sesetivity Adjustments
        self.asax = (((asax-128)*0.5)/128)+1
        self.asay = (((asay-128)*0.5)/128)+1
        self.asaz = (((asaz-128)*0.5)/128)+1

        # Set Register CNTL1 to 16-bit output, Continuous measurement mode 100Hz
        self.i2c.writeto_mem(0x0C, 0x0A, b'\x16') 

       
    def readMag( self, bias=(0,0,0,1,1,1,1,1,1) ):
        """
        return tuple of magnetometer measurements (x,y,z)
        North-East-Down(NED) as a fixed, parent coordinate system      
        """

        #Data Ready
        DRDY = self.i2c.readfrom_mem(0x0C, 0x02, 1)[0] & 0x01
 
        if DRDY != 0x01 : # Data is ready
            raise Exception()

        x,y,z = unpack('<hhh',self.i2c.readfrom_mem(0x0C, 0x03, 6))  

        HOFL = self.i2c.readfrom_mem(0x0C, 0x09, 1)[0] & 0x08

        # apply the Factory Magentometer Sensetivity adjustment
        x,y,z = x * self.asax, y * self.asay , z * self.asaz

        # apply offset
        x,y,z = x - bias[0], y - bias[1], z - bias[2]

        # apply normalize
        x,y,z = x / bias[3], y / bias[4], z / bias[5]

        # apply scale
        x,y,z = x * bias[6], y * bias[7], z * bias[8]

        return x,y,z

    def readMagHeading(self):
        '''returns the magnetic heading in degrees:  -179 -> 180 degrees'''
        x,y,_ = self.readMag( self.magbias )
        return int(degrees(atan2(x,y)))

    def calibrateMag( self, samples=800, delay=10 ):
        '''
        Creates a tuple of magbias and saves this to the imu store
        During the calibration rotate the gyro in all directions
        '''
        print("calibrate magnetmeter, by waving the robot around in a figure of 8")
        minx = 0
        maxx = 0
        miny = 0
        maxy = 0
        minz = 0
        maxz = 0

        while samples :

            samples = samples - 1
            try:
                x,y,z = self.readMag()  
                minx = min(x,minx)
                maxx = max(x,maxx)
                miny = min(y,miny)
                maxy = max(y,maxy)
                minz = min(z,minz)
                maxz = max(z,maxz)
            except Exception:
                pass
        
            print(x,y,z)
            utime.sleep_ms(delay)

        cx = (maxx + minx) / 2
        cy = (maxy + miny) / 2  
        cz = (maxz + minz) / 2  

        nx = abs(maxx - cx)
        ny = abs(maxy - cy)
        nz = abs(maxz - cz)

        # Soft iron correction
        avg_delta_x = (maxx - minx) / 2
        avg_delta_y = (maxy - miny) / 2
        avg_delta_z = (maxz - minz) / 2

        avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3
        
        sx = avg_delta / avg_delta_x
        sy = avg_delta / avg_delta_y
        sz = avg_delta / avg_delta_z

        print("offset",cx,cy,cz)
        print("normalisation",nx,ny,nz)
        print("scale",sx,sy,sz)

        self.magbias = (cx, cy, cz ,nx, ny, nz, sx, sy, sz) 
        self.save()

        return cx, cy, cz ,nx, ny, nz, sx, sy, sz

    #Temperature Sensor
    def readTemp( self ):
        """
        return temperature in deg Celcius
        """
        temp = unpack('>h',self.i2c.readfrom_mem(0x69, 0x41, 2)) 
        temp = ((temp[0] - self.tempoffset) / self.tempsensitivity) + 21

        return temp

    #Helper Functions
    def deltat(self):
        '''
        To assist integration
        Calculates for the differince in time between updates 
        '''
        currentTime = utime.ticks_us()

        if self.startTime is None:
            self.startTime = currentTime
            return 0.0001  # 100μs notional delay. 1st reading is invalid in any case

        deltaT =  utime.ticks_diff(currentTime,self.startTime )/1000000
        self.startTime = currentTime
        return deltaT


    def save(self, filename='imu.json' ):
        """write imu persistant data to flash"""
        import json
        with open(filename, 'w') as file:

            store = {}            
            store["accelbias"] = self.accelbias
            store["gyrobias"] = self.gyrobias
            store["magbias"] = self.magbias 
            # store["declination"] = self.declination
            store["tempoffset"] = self.tempoffset
            store["tempsensitivity"] = self.tempsensitivity

            json.dump(store, file)

    def load(self, filename='imu.json' ):
        """load imu persistant data set from flash"""
        import json 
        try:
            with open(filename, 'r') as file:
                
                store = json.load(file)

                self.accelbias = store["accelbias"]
                self.gyrobias = store["gyrobias"]
                self.magbias = store["magbias"]
                #self.declination = store["declination"]
                self.tempoffset = store["tempoffset"]
                self.tempsensitivity = store["tempsensitivity"]

        except Exception :
            pass
           

