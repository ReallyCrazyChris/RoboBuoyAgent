from math import floor, ceil


class SteeringPID():
    def __init__(self):

        # Pathfinding
        self.currentcourse = 0

        # PID tuning gains to control the steering based on desiredcourse vs currentcourse
        self.Kp = 1
        self.Ki = 0 #.5
        self.Kd = 0.5 #.0001

        # PID variables to matintain course by steering
        self.error = 0
        self.errSum = 0
        self.dErr = 0      
        self.lastErr = 0  # Previous error

        # Complimentary Filter tunings
        self.compassalpha = 0.97  # compasComplemt filter weigheted towards the gyro
        self.gpsalpha = 0.03      # gpsComplement filter weighted towards the gps


    def fusegyro(self,gyro_deg_s,deltaT ):
        #integrate
        self.currentcourse =  ( self.currentcourse + gyro_deg_s * deltaT )
        # clamp to -80 ... 180 degrees
        self.currentcourse = normalize(self.currentcourse,-180,180)
        return self.currentcourse

    def fusecompass(self, compasscourse):
        #fuse with a complement filter, strongly weighted towards the gyro
        self.currentcourse = (1.0 - self.compassalpha) * normalize(compasscourse,-180,180) + self.compassalpha * self.currentcourse
        self.currentcourse = normalize(self.currentcourse,-180,180)
        return self.currentcourse

    def fusegps(self, gpscourse):
        #fuse with a complement filter, strongly weighted towards the gps
        #TODO do we need normalize gps course
        self.currentcourse = (1.0 - self.gpsalpha) * normalize(gpscourse,-180,180) + self.gpsalpha * self.currentcourse
        self.currentcourse = normalize(self.currentcourse,-180,180)
        return self.currentcourse    

    def pidloop(self, desiredcourse, currentcourse, deltaT ):

        self.error  = desiredcourse - currentcourse
        self.errSum = self.errSum + (self.error * deltaT)
        self.dErr = (self.error - self.lastErr) / deltaT
    
        steering_angle = (self.Kp * self.error) + (self.Ki * self.errSum) + (self.Kd * self.dErr)
        
        self.lastErr = self.error

        return steering_angle 

def normalize(num, lower=0.0, upper=360.0, b=False):
    """ Got this code from : https://gist.github.com/phn/1111712/35e8883de01916f64f7f97da9434622000ac0390"""
   
    res = num
    if not b:
        if lower >= upper:
            raise ValueError("Invalid lower and upper limits: (%s, %s)" %
                             (lower, upper))

        res = num
        if num > upper or num == lower:
            num = lower + abs(num + upper) % (abs(lower) + abs(upper))
        if num < lower or num == upper:
            num = upper - abs(num - lower) % (abs(lower) + abs(upper))

        res = lower if res == upper else num
    else:
        total_length = abs(lower) + abs(upper)
        if num < -total_length:
            num += ceil(num / (-2 * total_length)) * 2 * total_length
        if num > total_length:
            num -= floor(num / (2 * total_length)) * 2 * total_length
        if num > upper:
            num = total_length - num
        if num < lower:
            num = -total_length - num

        res = num * 1.0  # Make all numbers float, to be consistent

    return res        
