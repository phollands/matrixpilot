#!/usr/bin/env python

'''
example program that dumps a Mavlink log file. The log file is
assumed to be in the format that qgroundcontrol uses, which consists
of a series of MAVLink packets, each with a 64 bit timestamp
header. The timestamp is in microseconds since 1970 (unix epoch)
'''

import sys, time, os, struct, math
import pylab as plt
import numpy as np
from Tkinter import *

# allow import from the parent directorfilty, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

from optparse import OptionParser
parser = OptionParser("mavlogdump.py [options]")

parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--planner",dest="planner", action='store_true', help="use planner file format")
parser.add_option("--robust",dest="robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_option("-f", "--follow",dest="follow", action='store_true', help="keep waiting for more data at end of file")
parser.add_option("--condition",dest="condition", default=None, help="select packets by condition")
parser.add_option("-q", "--quiet", dest="quiet", action='store_true', help="don't display packets")
parser.add_option("-o", "--output", default=None, help="output matching packets to give file")
parser.add_option("--types",  default=None, help="types of messages (comma separated)")
(opts, args) = parser.parse_args()

import mavutil

if len(args) < 1:
    print("Usage: mavlogdump.py [options] <LOGFILE>")
    sys.exit(1)

filename = args[0]
mlog = mavutil.mavlink_connection(filename, planner_format=opts.planner,
                                  notimestamps=opts.notimestamps,
                                  robust_parsing=opts.robust)

output = None
if opts.output:
    output = mavutil.mavlogfile(opts.output, write=True)

types = opts.types
if types is not None:
    types = types.split(',')
    
if output:
    output.write("msec, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag\n")    
    

# Converts relative degrees North to Latitude
def lat2meters(deg):
    return(deg * (1e7 / 89.983))

# Converts relative degrees East to Longitude
def lon2meters(deg, lat):
    return((deg * (1e7 / 89.983)) * math.cos(math.pi * lat / 180))

class boxcar:
    # run a boxcar integrator on raw IMU data: length 250msec
    atts = np.zeros(9)   
    index = 0
    length = 7
    buf = np.zeros((9, length), dtype=int)
    sums = np.zeros((9, 1), dtype=float)
    
    def update(self, data):
        
        # index of oldest value
        self.index += 1
        self.index %= self.length 
        
        for idx in range(0, 9):
            #print "boxIndex: ", self.index, " new: ", self.atts[idx], " old: ", self.buf[idx, self.index]
            # add (new - oldest) value
            self.sums[idx] += data[idx] - self.buf[idx, self.index]
            #print "sum: ", self.sums[idx]
            # save new value over oldest value
            self.buf[idx, self.index] = data[idx]
            
    def output(self):
        outdata = []
        for i in range(0, 9):
            outdata.append(float(boxcar.sums[i]) / boxcar.length)
        return outdata

class flight_log_book:
    def __init__(self) :
        self.raw_imu = [] # list of raw IMU data records
        self.filtered_imu = [] # list of filtered IMU data records
        self.attitude = []
        self.rc_channels = []
        self.servo_out = []
        self.f2a = []
        self.f2b = []
        self.F4 = "Empty"
        self.F5 = "Empty"
        self.F6 = "Empty"
        self.F7 = "Empty"
        self.F8 = "Empty"
        self.F11 = "Empty"
        self.F13 = []
        self.F14 = "Empty"
        self.F15 = "Empty"
        self.F16 = "Empty"
        self.ardustation_pos = "Empty"
        self.rebase_time_to_race_time = False
        self.waypoints_in_telemetry = False

        '''
        Backwards compatible MAVLink version of SERIAL_UDB_EXTRA - F2: Format
        Part A

        sue_time                  : Serial UDB Extra Time (uint32_t)
        systime_usec              : Serial UDB Extra systime (uint64_t)
        sue_status                : Serial UDB Extra Status (uint8_t)
        sue_latitude              : Serial UDB Extra Latitude (int32_t)
        sue_longitude             : Serial UDB Extra Longitude (int32_t)
        sue_altitude              : Serial UDB Extra Altitude (int32_t)
        sue_waypoint_index        : Serial UDB Extra Waypoint Index (uint16_t)
        sue_rmat0                 : Serial UDB Extra Rmat 0 (int16_t)
        sue_rmat1                 : Serial UDB Extra Rmat 1 (int16_t)
        sue_rmat2                 : Serial UDB Extra Rmat 2 (int16_t)
        sue_rmat3                 : Serial UDB Extra Rmat 3 (int16_t)
        sue_rmat4                 : Serial UDB Extra Rmat 4 (int16_t)
        sue_rmat5                 : Serial UDB Extra Rmat 5 (int16_t)
        sue_rmat6                 : Serial UDB Extra Rmat 6 (int16_t)
        sue_rmat7                 : Serial UDB Extra Rmat 7 (int16_t)
        sue_rmat8                 : Serial UDB Extra Rmat 8 (int16_t)
        sue_cog                   : Serial UDB Extra GPS Course Over Ground (uint16_t)
        sue_sog                   : Serial UDB Extra Speed Over Ground (int16_t)
        sue_cpu_load              : Serial UDB Extra CPU Load (uint16_t)
        sue_voltage_milis         : Serial UDB Extra Voltage in MilliVolts (int16_t)
        sue_air_speed_3DIMU        : Serial UDB Extra 3D IMU Air Speed (uint16_t)
        sue_estimated_wind_0        : Serial UDB Extra Estimated Wind 0 (int16_t)
        sue_estimated_wind_1        : Serial UDB Extra Estimated Wind 1 (int16_t)
        sue_estimated_wind_2        : Serial UDB Extra Estimated Wind 2 (int16_t)
        sue_magFieldEarth0        : Serial UDB Extra Magnetic Field Earth 0 (int16_t)
        sue_magFieldEarth1        : Serial UDB Extra Magnetic Field Earth 1 (int16_t)
        sue_magFieldEarth2        : Serial UDB Extra Magnetic Field Earth 2 (int16_t)
        sue_svs                   : Serial UDB Extra Number of Sattelites in View (int16_t)
        sue_hdop                  : Serial UDB Extra GPS Horizontal Dilution of Precision (int16_t)

        '''
        '''
        Backwards compatible version of SERIAL_UDB_EXTRA - F2: Part B

        sue_time                  : Serial UDB Extra Time (uint32_t)
        systime_usec              : Serial UDB Extra systime (uint64_t)
        sue_pwm_input_1           : Serial UDB Extra PWM Input Channel 1 (int16_t)
        sue_pwm_input_2           : Serial UDB Extra PWM Input Channel 2 (int16_t)
        sue_pwm_input_3           : Serial UDB Extra PWM Input Channel 3 (int16_t)
        sue_pwm_input_4           : Serial UDB Extra PWM Input Channel 4 (int16_t)
        sue_pwm_input_5           : Serial UDB Extra PWM Input Channel 5 (int16_t)
        sue_pwm_input_6           : Serial UDB Extra PWM Input Channel 6 (int16_t)
        sue_pwm_input_7           : Serial UDB Extra PWM Input Channel 7 (int16_t)
        sue_pwm_input_8           : Serial UDB Extra PWM Input Channel 8 (int16_t)
        sue_pwm_input_9           : Serial UDB Extra PWM Input Channel 9 (int16_t)
        sue_pwm_input_10          : Serial UDB Extra PWM Input Channel 10 (int16_t)
        sue_pwm_output_1          : Serial UDB Extra PWM Output Channel 1 (int16_t)
        sue_pwm_output_2          : Serial UDB Extra PWM Output Channel 2 (int16_t)
        sue_pwm_output_3          : Serial UDB Extra PWM Output Channel 3 (int16_t)
        sue_pwm_output_4          : Serial UDB Extra PWM Output Channel 4 (int16_t)
        sue_pwm_output_5          : Serial UDB Extra PWM Output Channel 5 (int16_t)
        sue_pwm_output_6          : Serial UDB Extra PWM Output Channel 6 (int16_t)
        sue_pwm_output_7          : Serial UDB Extra PWM Output Channel 7 (int16_t)
        sue_pwm_output_8          : Serial UDB Extra PWM Output Channel 8 (int16_t)
        sue_pwm_output_9          : Serial UDB Extra PWM Output Channel 9 (int16_t)
        sue_pwm_output_10         : Serial UDB Extra PWM Output Channel 10 (int16_t)
        sue_imu_location_x        : Serial UDB Extra IMU Location X (int16_t)
        sue_imu_location_y        : Serial UDB Extra IMU Location Y (int16_t)
        sue_imu_location_z        : Serial UDB Extra IMU Location Z (int16_t)
        sue_flags                 : Serial UDB Extra Status Flags (uint32_t)
        sue_osc_fails             : Serial UDB Extra Oscillator Failure Count (int16_t)
        sue_imu_velocity_x        : Serial UDB Extra IMU Velocity X (int16_t)
        sue_imu_velocity_y        : Serial UDB Extra IMU Velocity Y (int16_t)
        sue_imu_velocity_z        : Serial UDB Extra IMU Velocity Z (int16_t)
        sue_waypoint_goal_x        : Serial UDB Extra Current Waypoint Goal X (int16_t)
        sue_waypoint_goal_y        : Serial UDB Extra Current Waypoint Goal Y (int16_t)
        sue_waypoint_goal_z        : Serial UDB Extra Current Waypoint Goal Z (int16_t)
        sue_memory_stack_free        : Serial UDB Extra Stack Memory Free (int16_t)

        '''
        '''
        Backwards compatible version of SERIAL_UDB_EXTRA F13: format

        sue_week_no               : Serial UDB Extra GPS Week Number (int16_t)
        sue_lat_origin            : Serial UDB Extra MP Origin Latitude (int32_t)
        sue_lon_origin            : Serial UDB Extra MP Origin Longitude (int32_t)
        sue_alt_origin            : Serial UDB Extra MP Origin Altitude Above Sea Level (int32_t)

        '''

    def plot_imu(self):
        
        f2a_t = np.zeros([len(self.f2a), 1])
        sogdata = np.zeros([len(self.f2a), 1])
        latdata = np.zeros([len(self.f2a), 1])
        londata = np.zeros([len(self.f2a), 1])
        northdata = np.zeros([len(self.f2a), 1])
        eastdata = np.zeros([len(self.f2a), 1])
        altdata = np.zeros([len(self.f2a), 1])
        
        lat_origin = 0
        lon_origin = 0
        if (len(self.F13) > 0): 
            lat_origin = 1e-7 * self.F13[-1].sue_lat_origin
            lon_origin = 1e-7 * self.F13[-1].sue_lon_origin
            alt_origin = 1e-2 * self.F13[-1].sue_alt_origin
            print("plot_imu: using origin (%s, %s, %s)" % (lat_origin, lon_origin, alt_origin))
            
        recN = 0
        for msg in self.f2a:
            f2a_t[recN] = msg.systime_usec / 1.0e6
            if (msg.sue_latitude != 0) and (msg.sue_longitude != 0):
                sogdata[recN] = msg.sue_sog
                latdata[recN] = 1e-7 * msg.sue_latitude - lat_origin
                northdata[recN] = lat2meters(latdata[recN])
                londata[recN] = 1e-7 * msg.sue_longitude - lon_origin
                eastdata[recN] = lon2meters(londata[recN], latdata[recN])
                altdata[recN] = 1e-2 * msg.sue_altitude - alt_origin
            recN += 1            
        
        # fieldnames = ['time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag']
        imudata = np.array(self.raw_imu)
        imudata_filt = np.array(self.filtered_imu)
        
        pwmIn = np.array(self.rc_channels)
        pwmOut = np.array(self.servo_out)
        attdata = np.array(self.attitude)
        
        att_t = attdata[0:,0] / 1.0e6
        roll = attdata[0:,1]
        pitch = attdata[0:,2] * 20
        yaw = attdata[0:,3]

        imu_t = imudata[0:,0] / 1.0e6
        raw_xacc = imudata[0:,1]
        raw_yacc = imudata[0:,2]
        raw_zacc = imudata[0:,3]
        raw_zgyro = imudata[0:,6]
        raw_accmag = np.zeros((len(self.raw_imu), 1))
        for i in np.arange(len(self.raw_imu)):
            raw_accmag[i] = math.sqrt(math.pow(raw_xacc[i], 2) + 
                                      math.pow(raw_yacc[i], 2) + 
                                      math.pow(raw_zacc[i], 2))
        imu_filt_t = imudata_filt[0:,0] / 1.0e6
        filt_xacc = imudata_filt[0:,1]
        filt_zacc = imudata_filt[0:,3]
        
        pwmIn_t = pwmIn[0:, 0] / 1.0e6
        pwmOut_t = pwmOut[0:, 0] / 1.0e6
        rudder_man = 20 * (pwmIn[0:,4] - 1500)
        mode = 20 * (pwmIn[0:,6] - 1500)
        rudder_out = 20 * (pwmOut[0:,4] - 1500)
        aileron_out = 20 * (pwmOut[0:,2] - 1500)
        throttle_out = 20 * (pwmOut[0:,1] - 1500)
        bomb_out = 20 * (pwmOut[0:,5] - 1500)
        
        plt.figure(1)
        plt.title('''lateral acceleration feedback''')
#         plt.plot(imu_t, raw_xacc, 'o', mew=0.0, label='xacc')
        plt.plot(imu_filt_t, filt_xacc, '-o', mew=0.0, label='xacc_filt')
#         plt.plot(imu_filt_t, filt_zacc, 'o', mew=0.0, label='zacc_filt')
        plt.plot(pwmOut_t, rudder_out, '-o', mew=0.0, label='rudder_out')
#         plt.plot(pwmIn_t, rudder_man, '-o', mew=0.0, label='rudder_man')
        plt.plot(imu_t, raw_zgyro, '-o', mew=0.0, label='yaw rate')
        plt.plot(pwmIn_t, mode, '-o', mew=0.0, label='mode')
        plt.plot(pwmOut_t, aileron_out, '-o', mew=0.0, label='aileron_out')
        plt.plot(pwmOut_t, bomb_out, '-o', mew=0.0, label='bomb release')
        plt.plot(att_t, pitch, '-o', mew=0.0, label='pitch')
        plt.plot(f2a_t, altdata, '-o', mew=0.0, label='altitude cm')
        plt.plot(f2a_t, sogdata, '-o', mew=0.0, label='SOG cm/sec')
        plt.xlabel('system time: sec')
        plt.ylabel('xacc')
        plt.grid()
        plt.legend()
    
        plt.figure(2)
        plt.title('raw accelerometer data')
        plt.plot(imu_t, raw_xacc, '-o', mew=0.0, label='xacc')
        plt.plot(imu_t, raw_yacc, '-o', mew=0.0, label='yacc')
        plt.plot(imu_t, raw_zacc, '-o', mew=0.0, label='zacc')
        plt.plot(imu_t, raw_accmag, '-o', mew=0.0, label='mag')
#         plt.plot(imu_t, raw_zgyro, '-o', mew=0.0, label='yaw rate')
# 
#         plt.plot(pwmOut_t, throttle_out, '-o', mew=0.0, label='throttle')
#         plt.plot(pwmOut_t, rudder_out, '-o', mew=0.0, label='rudder_out')
        
        plt.xlabel('system time: sec')
        plt.ylabel('acc')
        plt.grid()
        plt.legend()
        
#         plt.figure(3)
#         plt.title('Sport Cub: adverse yaw')
#         plt.plot(imu_t, raw_zgyro, '-o', mew=0.0, label='yaw rate')
#         plt.plot(pwmOut_t, aileron_out, '-o', mew=0.0, label='aileron_out')
#         plt.plot(pwmOut_t, rudder_out, '-o', mew=0.0, label='rudder_out')
#         
#         plt.xlabel('system time: sec')
#         plt.ylabel('acc')
#         plt.grid()
#         plt.legend()

        plt.figure(3)
        plt.title('LEA-6H lat/lon')
        plt.plot(eastdata, northdata, '-o', mew=0.0, label='lat/lon')
        
        plt.xlabel('east meters')
        plt.ylabel('north meters')
        plt.grid()
        plt.legend()
        plt.axis('equal')
        
        plt.figure(4)
        plt.title('LEA-6H lat/lon')
        plt.plot(f2a_t, northdata, '-o', mew=0.0, label='north')
        plt.plot(f2a_t, eastdata, '-o', mew=0.0, label='east')
        
        plt.xlabel('seconds')
        plt.ylabel('degrees')
        plt.grid()
        plt.legend()
        
        plt.show(block=False)

########## Start of the Main Program ##########
    
if __name__=="__main__":

#     root = Tk()
#     root.title("Flight Analyzer")
#     w = Canvas(root, width=600, height=300)

    log_book = flight_log_book()
    boxfilter = boxcar()
    last_F2A_msg = None
    last_timestamp = 0
    while True:
        msg = mlog.recv_match(condition=opts.condition, blocking=opts.follow)
        if msg is None:
            break
    
        if types is not None and msg.get_type() not in types:
            continue
        if output and (msg.get_type() != 'BAD_DATA'):
            if hasattr(msg, 'time_boot_ms'):
                last_timestamp = 1000 * msg.time_boot_ms
            elif hasattr(msg, 'time_usec'):
                last_timestamp = msg.time_usec
                    
#             print "timestamp: %i, type: %s" % (last_timestamp, msg.get_type())
                
            if msg.get_type() == 'ATTITUDE':
                entry = [1000 * msg.time_boot_ms, 
                         msg.roll * 180/math.pi, 
                         msg.pitch * 180/math.pi, 
                         msg.yaw * 180/math.pi]
                log_book.attitude.append(entry)
                
            elif msg.get_type() == 'RAW_IMU':
                boxfilter.update([msg.xacc, msg.yacc, msg.zacc,
                                  msg.xgyro, msg.ygyro, msg.zgyro, 
                                  msg.xmag, msg.ymag, msg.zmag])
                
                entry = [msg.time_usec, 
                         msg.xacc, msg.yacc, msg.zacc,
                         msg.xgyro, msg.ygyro, msg.zgyro, 
                         msg.xmag, msg.ymag, msg.zmag]
                log_book.raw_imu.append(entry)
                
                filterTime = msg.time_usec - 20000 * (math.floor(boxcar.length/2.0))
                entry = [filterTime] + boxfilter.output()
                log_book.filtered_imu.append(entry)
                
            elif msg.get_type() == 'SERIAL_UDB_EXTRA_F2_A':
                log_book.f2a.append(msg)
                
            elif msg.get_type() == 'SERIAL_UDB_EXTRA_F2_B':
                log_book.f2b.append(msg)
                    
            elif msg.get_type() == 'SERIAL_UDB_EXTRA_F13':
                log_book.F13.append(msg)
                print('found SUE_F13 message: origin: (%s, %s, %s)' % 
                      (msg.sue_lat_origin, msg.sue_lon_origin, msg.sue_alt_origin))
                    
            elif msg.get_type() == 'RC_CHANNELS_RAW':
                if (msg.port == 220):
                    entry = [1000 * msg.time_boot_ms, 
                             msg.chan1_raw,
                             msg.chan2_raw,
                             msg.chan3_raw,
                             msg.chan4_raw,
                             msg.chan5_raw,
                             msg.chan6_raw,
                             msg.chan7_raw,
                             msg.chan8_raw]
                    log_book.rc_channels.append(entry)
                
            elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                if (msg.port == 220):
                    entry = [1000 * msg.time_boot_ms, 
                             msg.servo1_raw,
                             msg.servo2_raw,
                             msg.servo3_raw,
                             msg.servo4_raw,
                             msg.servo5_raw,
                             msg.servo6_raw,
                             msg.servo7_raw,
                             msg.servo8_raw]
                    log_book.servo_out.append(entry)
     
        else:
            print "bad data following timestamp: ", last_timestamp
            print msg
    
    log_book.plot_imu()
    mainloop()    
    
#     root.quit()
