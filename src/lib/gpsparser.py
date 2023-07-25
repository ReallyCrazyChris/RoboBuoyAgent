import time
import uasyncio as asyncio
from math import sin, cos, sqrt, atan2, degrees, radians

__HEMISPHERES = ('N', 'S', 'E', 'W')

class GPS(object):

    """GPS NMEA Sentence Parser. Creates object that stores all relevant GPS data and statistics.
    Parses sentences one character at a time using update(). """

        # Max Number of Characters a valid sentence can be (based on GGA sentence)
    

    def __init__(self, local_offset=0):
        """Setup GPS Object Status Flags, Internal Data Registers, etc"""

        #####################
        # Object Status Flags
        self.sentence_active = False
        self.active_segment = 0
        self.process_crc = False
        self.gps_segments = []
        self.crc_xor = 0
        self.char_count = 0
        self.fix_time = 0

        #####################
        # Sentence Statistics
        self.crc_fails = 0
        self.clean_sentences = 0
        self.parsed_sentences = 0

        #####################
        # Data From Sentences
        # Time
        self.timestamp = (0, 0, 0)
        self.date = (0, 0, 0)
        self.local_offset = local_offset

        # Position/Motion
        self.positionvalid = False
        self.timestamp = (0, 0, 0)
        self.latitude = 0
        self.longitude = 0
        self.latitude_string = ''
        self.longitude_string = ''
        self.position = (0,0)
        self.speed = 0.0  #meters per second
        self.course = 0.0 #degrees

        # Pathfinding
        self.waypoints = [(49.69395, 10.82761)]

        # UART readall
        self.oldstring = bytes()
        
    ########################################
    # Sentence Parsers
    ########################################
    
    def gpgll(self):
        """Parse Geographic Latitude and Longitude (GLL)Sentence. Updates UTC timestamp, latitude,
        longitude, and fix status"""

        # UTC Timestamp
        try:
            utc_string = self.gps_segments[5]

            if utc_string:  # Possible timestamp found
                hours = int(utc_string[0:2]) + self.local_offset
                minutes = int(utc_string[2:4])
                seconds = float(utc_string[4:])
                self.timestamp = (hours, minutes, seconds)

            else:  # No Time stamp yet
                self.timestamp = (0, 0, 0)

        except ValueError:  # Bad Timestamp value present
            return False

        # Check Receiver Data Valid Flag
        if self.gps_segments[6] == 'A':  # Data from Receiver is Valid/Has Fix

            # Longitude / Latitude
            try:
                # Latitude
                l_string = self.gps_segments[1]
                lat_degs = l_string[0:2]
                lat_mins = l_string[2:]
                lat_hemi = self.gps_segments[2]
                
                # Longitude
                l_string = self.gps_segments[3]
                lon_degs = l_string[0:3]
                lon_mins = l_string[3:]
                lon_hemi = self.gps_segments[4]

                if lat_hemi not in __HEMISPHERES:
                    raise ValueError()

                if lon_hemi not in __HEMISPHERES:
                    raise ValueError()                    

                self.latitude, self.latitude_string = self.convert_dm_dd(lat_degs, lat_mins, lat_hemi)
                self.longitude, self.longitude_string = self.convert_dm_dd(lon_degs, lon_mins, lon_hemi)  
                self.position = (self.latitude, self.longitude)

            except ValueError:
                return False

            self.positionvalid = True

            # Update Last Fix Time
            self.new_fix_time()

        else:  # Clear Position Data if Sentence is 'Invalid'
            self.latitude = 0
            self.latitude_string = '0'
            self.longitude = 0
            self.longitude_string ='0'
            self.position = (0,0)

            self.positionvalid = False

        return True

    def gpvtg(self):
        """Parse Track Made Good and Ground Speed (VTG) Sentence. Updates speed and course"""
        try:
            course = float(self.gps_segments[1])
            spd_knt = float(self.gps_segments[5])
        except ValueError:
            return False

        # Include mph and km/h
        self.speed = spd_knt
        self.course = course
        return True

    ##########################################
    # Data Stream Handler Functions
    ##########################################

    def new_sentence(self):
        """Adjust Object Flags in Preparation for a New Sentence"""
        self.gps_segments = ['']
        self.active_segment = 0
        self.crc_xor = 0
        self.sentence_active = True
        self.process_crc = True
        self.char_count = 0

    def parsesentence(self, string):
        idx = 0        
        string_tmp = self.oldstring + string
        
        for c in string_tmp:            
            idx = idx + 1
            stat = self.update(chr(c))                        
            if(stat != None):                                                
                self.oldstring = string_tmp[idx:]
                return stat
            
    def stringclean(self):
        self.oldstring = bytes()        

    def update(self, new_char):
        """Process a new input char and updates GPS object if necessary based on special characters ('$', ',', '*')
        Function builds a list of received string that are validate by CRC prior to parsing by the  appropriate
        sentence function. Returns sentence type on successful parse, None otherwise"""

        valid_sentence = False

        # Validate new_char is a printable char        
        ascii_char = ord(new_char)

        if 10 <= ascii_char <= 126:
            self.char_count += 1

            # Check if a new string is starting ($)
            if new_char == '$':
                self.new_sentence()
                return None

            elif self.sentence_active:

                # Check if sentence is ending (*)
                if new_char == '*':
                    self.process_crc = False
                    self.active_segment += 1
                    self.gps_segments.append('')
                    return None

                # Check if a section is ended (,), Create a new substring to feed
                # characters to
                elif new_char == ',':
                    self.active_segment += 1
                    self.gps_segments.append('')

                # Store All Other printable character and check CRC when ready
                else:
                    self.gps_segments[self.active_segment] += new_char

                    # When CRC input is disabled, sentence is nearly complete
                    if not self.process_crc:

                        if len(self.gps_segments[self.active_segment]) == 2:
                            try:
                                final_crc = int(self.gps_segments[self.active_segment], 16)
                                if self.crc_xor == final_crc:
                                    valid_sentence = True
                                else:
                                    self.crc_fails += 1
                            except ValueError:
                                pass  # CRC Value was deformed and could not have been correct

                # Update CRC
                if self.process_crc:
                    self.crc_xor ^= ascii_char

                # If a Valid Sentence Was received and it's a supported sentence, then parse it!!
                if valid_sentence:
                    self.clean_sentences += 1  # Increment clean sentences received
                    self.sentence_active = False  # Clear Active Processing Flag

                    if self.gps_segments[0][2:] in self.supported_sentences:
                        # parse the Sentence Based on the message type, return True if parse is clean                        
                        if self.supported_sentences[self.gps_segments[0][2:]](self):                        
                            # Let host know that the GPS object was updated by returning parsed sentence type
                            self.parsed_sentences += 1
                            return self.gps_segments[0]

                # Check that the sentence buffer isn't filling up with Garage waiting for the sentence to complete
                if self.char_count > 76:
                    self.sentence_active = False

        # Tell Host no new sentence was parsed
        return None

    def new_fix_time(self):
        """Updates a high resolution counter with current time when fix is updated. Currently only triggered from
        GGA, GSA and RMC sentences"""
        self.fix_time = time.time()
        return self.fix_time



    def distance(self,p1:tuple,p2:tuple) -> int:    
        """
        distance in meters between 2 position
        p1 = lat_dd,lon_dd degree decimal format
        p2 = lat_dd,lon_dd degree decimal format
        returns int distans in meters
        """
        
        R = 6373000        # Radius of the earth in m
        
        lat1_dd, lon1_dd = p1
        lat1_dd, long1_dd = radians(lat1_dd), radians(lon1_dd)

        lat2_dd, lon2_dd = p2
        lat2_dd, lon2_dd = radians(lat2_dd), radians(lon2_dd)
        
        deltaLat = lat2_dd - lat1_dd
        deltaLon = lon2_dd - long1_dd
        
        x = deltaLon * cos((lat1_dd+lat2_dd)/2)
        distance = sqrt(x**2 + deltaLat**2) * R
        
        return distance

    def bearing(self, p1:tuple, p2:tuple) -> int:
        """
        provides a bearing between two positions
        p1 = (lat_dd, lon_dd) degree decimal format
        p2 = (lat_dd, lon_dd) degree decimal format
        """
        lat1_dd, lon1_dd = p1
        lat1_dd, lon1_dd = radians(lat1_dd), radians(lon1_dd)

        lat2_dd, lon2_dd = p2
        lat2_dd, lon2_dd = radians(lat2_dd), radians(lon2_dd)
        
        deltaLon = lon2_dd - lon1_dd
        
        y = sin(deltaLon) * cos(lat2_dd)
        x = cos(lat1_dd) * sin(lat2_dd) - sin(lat1_dd) * cos(lat2_dd) * cos(deltaLon)
        
        bearing = (degrees(atan2(y, x)) + 360) % 360
        return bearing

    def convert_dm_dd(self, degree :str,minutes :str, hemi :str) -> tuple:
        """ 
        convert degree minutes format to degrees decimal format 
        eg 49 21.3454 S -> dd = -49.3557566
        returns float and string representations of degree decimal
        ISSUE# On small mcu's the float precision is low:
            eg. '49.3557566' -> 49.35575 
            this can cause the robot hunt or occilate around a waypoint
        """
        degree = int(degree)
        minuite, minuite_decimal = minutes.split('.')
        degree_decimal  = int(minuite + minuite_decimal) // 6

        if hemi in ['S','W']:
            degree=degree * -1

        dd_str = str(degree)+'.'+str(degree_decimal)
        dd_float = float(dd_str)

        return (dd_float, dd_str)

    # The supported NMEA sentences    
    supported_sentences = { 'VTG': gpvtg,  'GLL': gpgll } # GPS + GLONASS