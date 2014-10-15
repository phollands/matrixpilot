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

# allow import from the parent directory, where mavlink.py is
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
    

class boxcar:
    # run a boxcar integrator on raw IMU data: length 250msec
    atts = np.zeros(9)   
    index = 0
    length = 25
    buf = np.zeros((9, 25), dtype=int)
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
        self.F13 = "Empty"
        self.F14 = "Empty"
        self.F15 = "Empty"
        self.F16 = "Empty"
        self.ardustation_pos = "Empty"
        self.rebase_time_to_race_time = False
        self.waypoints_in_telemetry = False

    def plot_imu(self):
    #         sys.argv = ['junk']
        # fieldnames = ['time_usec', 'xacc', 'yacc', 'zacc', 'xgyro', 'ygyro', 'zgyro', 'xmag', 'ymag', 'zmag']
        imudata = np.array(self.raw_imu)
        imudata_filt = np.array(self.filtered_imu)
        pwmIn = np.array(self.rc_channels)
        pwmOut = np.array(self.servo_out)
#         pwmIn = np.zeros((len(self.f2b), 9))
#         pwmOut = np.zeros((len(self.f2b), 9))
#         index = 0
#         for entry in self.f2b:
#             pwmIn[index, 0] = entry.systime_usec
#             pwmIn[index, 1] = entry.sue_pwm_input_1
#             pwmIn[index, 2] = entry.sue_pwm_input_2
#             pwmIn[index, 3] = entry.sue_pwm_input_3
#             pwmIn[index, 4] = entry.sue_pwm_input_4
#             pwmIn[index, 5] = entry.sue_pwm_input_5
#             pwmIn[index, 6] = entry.sue_pwm_input_6
#             pwmIn[index, 7] = entry.sue_pwm_input_7
#             pwmIn[index, 8] = entry.sue_pwm_input_8
# 
#             pwmOut[index, 0] = entry.systime_usec
#             pwmOut[index, 1] = entry.sue_pwm_output_1
#             pwmOut[index, 2] = entry.sue_pwm_output_2
#             pwmOut[index, 3] = entry.sue_pwm_output_3
#             pwmOut[index, 4] = entry.sue_pwm_output_4
#             pwmOut[index, 5] = entry.sue_pwm_output_5
#             pwmOut[index, 6] = entry.sue_pwm_output_6
#             pwmOut[index, 7] = entry.sue_pwm_output_7
#             pwmOut[index, 8] = entry.sue_pwm_output_8
#                 
#             index+=1

        imu_t = imudata[0:,0]
        raw_xacc = -imudata[0:,1]
        raw_yacc = -imudata[0:,2]
        raw_zacc = -imudata[0:,3]
        raw_accmag = np.zeros((len(self.raw_imu), 1))
        for i in np.arange(len(self.raw_imu)):
            raw_accmag[i] = math.sqrt(math.pow(raw_xacc[i], 2) + 
                                      math.pow(raw_yacc[i], 2) + 
                                      math.pow(raw_zacc[i], 2))
        imu_filt_t = imudata_filt[0:,0]
        filt_xacc = -imudata_filt[0:,1]
        filt_zacc = imudata_filt[0:,3]
        
        pwmIn_t = pwmIn[0:, 0]
        pwmOut_t = pwmOut[0:, 0]
        rudder_out = 20 * (pwmOut[0:,4] - 1500)
        rudder_man = 20 * (pwmIn[0:,4] - 1500)
        mode = 20 * (pwmIn[0:,6] - 1500)
        aileron_out = 20 * (pwmOut[0:,2] - 1500)
        
        plt.figure(1)
        plt.title('''markw's custom plot''')
        plt.plot(imu_t, raw_xacc, 'o', mew=0.0, label='xacc')
        plt.plot(imu_filt_t, filt_xacc, '-o', mew=0.0, label='xacc_filt')
        plt.plot(imu_filt_t, filt_zacc, 'o', mew=0.0, label='zacc_filt')
        plt.plot(pwmOut_t, rudder_out, '-o', label='rudder_out')
        plt.plot(pwmIn_t, rudder_man, '-o', label='rudder_man')
        plt.plot(pwmIn_t, mode, '-o', label='mode')
        plt.plot(pwmOut_t, aileron_out, '-o', label='aileron_out')
        plt.xlabel('system time: usec')
        plt.ylabel('xacc')
        plt.grid()
        plt.legend()
    
        plt.figure(2)
        plt.title('raw accelerometer data')
        plt.plot(imu_t, raw_xacc, '-o', mew=0.0, label='xacc')
        plt.plot(imu_t, raw_yacc, '-o', mew=0.0, label='yacc')
        plt.plot(imu_t, raw_zacc, '-o', mew=0.0, label='zacc')
        plt.plot(imu_t, raw_accmag, '-o', mew=0.0, label='mag')
        plt.xlabel('system time: usec')
        plt.ylabel('acc')
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
                    
#             print "type: %s, timestamp: %i" % (msg.get_type(), last_timestamp)
                
            if msg.get_type() == 'ATTITUDE':
                entry = [msg.time_boot_ms, 
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
                
                filterTime = msg.time_usec - 250000
                entry = [filterTime] + boxfilter.output()
                log_book.filtered_imu.append(entry)
                
            elif msg.get_type() == 'SERIAL_UDB_EXTRA_F2_A':
                log_book.f2a.append(msg)
                
            elif msg.get_type() == 'SERIAL_UDB_EXTRA_F2_B':
                log_book.f2b.append(msg)
                    
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
