'''
Created on Oct 24, 2014

@author: markw
'''
from __builtin__ import file
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
import sys

if __name__ == '__main__':
    file = open('./ipldata/ipl.csv', 'ro')
    ipl_ofile = open('./ipldata/iplvals.csv', 'w')
    sp_ofile = open('./ipldata/spvals.csv', 'w')
    chData = []
    lnumber = 0
    for line in file:
        fields = line.split(';')
        nChan = len(fields) - 1
        lnumber += 1
        if (lnumber == 1): 
            header = fields
            for i in range(0, nChan+1):
                chData.append([])
        else:
            timeFields = fields[0].split(' ')
            units = timeFields[-1]
            # units is one of 'ns', 'us', 'ms', 's'
            if (units == 'ns'):
                t = float(timeFields[0]) * 1e-9
            elif (units == 'us'):
                t = float(timeFields[0]) * 1e-6
            elif (units == 'ms'):
                t = float(timeFields[0] + timeFields[1]) * 1e-3
            elif (units == 's'):
                t = float(timeFields[0] + timeFields[1] + timeFields[2])
            
            chData[0].append(t)
            for i in range(1, nChan+1):
                chData[i].append(int(fields[i]))
    
    # convert lists to arrays
    nRecs = len(chData[0])
    tvals = np.array(chData[0])
    
    # general array of data channels
    dvals = np.array(chData[1:nChan+1])
    
    # specific case for IPL analysis
    # in the "new" logic analyzer capture, the first 3 data channels are IPL level [lsb:msb]
    # the 4th channel is PWM output 1
    # channels [5:7] are the IPL stack level, [lsb:msb]
    iplVals = np.zeros(nRecs, dtype='uint8')
    pwm1 = np.zeros(nRecs, dtype='uint8')
    spVals = np.zeros(nRecs, dtype='uint8')
    for i in range(0, nRecs):
        iplVals[i] = dvals[0,i] + 2 * dvals[1,i] + 4 * dvals[2,i]
        pwm1[i] = dvals[3,i]
        spVals[i] = dvals[4,i] + 2 * dvals[5,i] + 4 * dvals[6,i]
        
    ipl_ofile.write("time,duration,IPL\n")
    sp_ofile.write("time,duration,nestlevel\n")
        
    iplAccum = np.zeros(8)
    iplCount = np.zeros(8)
    mindt = np.zeros(8)
    maxdt = np.zeros(8)
    maxdt_time = np.zeros(8)
    last_start_time = np.zeros(8)    

    duration = tvals[-1] - tvals[0]   
    for i in range(0, 8):
        mindt[i] = duration
        maxdt[i] = 0
    
    # histograms
    nbins = 50
    bin_width = 5e-6
    duration_histo = np.zeros((8, nbins))
    
    ibin_width = .1e-3
    interval_histo = np.zeros((8, nbins))
    
    traceTime = []
    tracedt = []
    traceIPL = []
    prevIPL = iplVals[0]
    lastIPLtransition = tvals[0]

    # i is a counter of discrete events representing each change in the data vector 
    for i in range(1, nRecs):
        # if IPLvals has changed, record info on prevIPL
        if (iplVals[i] != prevIPL):
            dt = tvals[i] - lastIPLtransition
            if dt < 100e-9:
                # prevIPL is a glitch, throw it away
                prevIPL = iplVals[i]
            else:
                # this is a valid transition
                # generate lists of valid IPL times, values and durations
                traceTime.append(lastIPLtransition)
                tracedt.append(dt)
                traceIPL.append(prevIPL)
                ipl_ofile.write("%10.9f,%10.9f,%d\n" % (lastIPLtransition, dt, prevIPL))
        
                # keep track of min/max dt
                if dt < mindt[prevIPL]: 
                    mindt[prevIPL] = dt
                    
                if dt > maxdt[prevIPL]: 
                    maxdt[prevIPL] = dt
                    maxdt_time[prevIPL] = tvals[i]
                    
                # track total time at each IPL
                iplAccum[prevIPL] += dt
                iplCount[prevIPL] += 1
                
                # histogram the durations
                bin_num = dt / bin_width
                if bin_num >= nbins: bin_num = nbins-1
                duration_histo[prevIPL, bin_num] += 1
                
                # histogram the intervals
                if last_start_time[prevIPL] != 0:
                    tint = tvals[i] - last_start_time[prevIPL]
                    bin_num = tint / ibin_width
                    if bin_num >= nbins: bin_num = nbins-1
                    interval_histo[prevIPL, bin_num] += 1
                
                # record last interval start time for each level
                last_start_time[prevIPL] = lastIPLtransition
                # update to new IPL level
                prevIPL = iplVals[i]
                lastIPLtransition = tvals[i]
            
        
    # deglitch the stack level data
    traceSP = []
    prevSP = spVals[0]
    lastSPtransition = tvals[0]
    for i in range(1, nRecs):
        # if spVals has changed, record info on prevSP
        if (spVals[i] != prevSP):
            dt = tvals[i] - lastSPtransition
            if dt < 500e-9:
                # prevSP is a glitch
                prevSP = spVals[i]
            else:           
                traceSP.append(prevSP)
                sp_ofile.write("%10.9f,%10.9f,%d\n" % (lastSPtransition, dt, prevSP))
                # update to new SP level
                prevSP = spVals[i]
                lastSPtransition = tvals[i]

    trace_SP = np.array(traceSP)
    maxSP = np.max(trace_SP)
    maxspcnt = 0
    for i in range(trace_SP.size):
        if trace_SP[i] == maxspcnt:
            maxspcnt += 1
        
    print("max number of active ISRs: %d, events: %d" % (maxSP, maxspcnt))
        
    avgIntvl = duration / nRecs
    print("avg interrupt interval: %5.3f usec, %d cycles" % (1e6 * avgIntvl, 70e6 * avgIntvl))
    
    print("busy percentages")
    print("IPL    %   avg dur usec  min      max    time      count   freq Hz")
    for i in range(0, 8):
        if iplCount[i] > 0:
            print("%d    %5.1f    %5.1f    %5.1f    %5.1f %10.5f   %5d   %5.3f" % 
                  (i, 100 * iplAccum[i] / duration, 1e6 * iplAccum[i] / iplCount[i],
                   1e6 * mindt[i], 1e6 * maxdt[i], maxdt_time[i], iplCount[i], iplCount[i] / duration))
            
    iStart = 0
    while traceTime[iStart] < (maxdt_time[3] - .020): iStart+=1
    iEnd = 0
    while traceTime[iEnd] < (maxdt_time[3] + .020): iEnd+=1
    
    trace_t = np.array(traceTime[iStart:iEnd])
    trace_dt = np.array(tracedt[iStart:iEnd])
    trace_ipl = np.array(traceIPL[iStart:iEnd])            
    fig, ax = plt.subplots(figsize=(20, 5))
    ax.bar(trace_t, trace_ipl, trace_dt, edgecolor='none')
    plt.show
        
    # plot histograms
    ind = np.arange(nbins)
    values = range(8)

    bar_width = 1.0/8
    fig, (ax1, ax2) = plt.subplots(1,2,figsize=(20, 5))
    ax1.set_title('ISR duration')
    ax2.set_title('ISR interval')
    ax1.set_xlabel('bin width %5.1f usec' % (1e6 * bin_width))
    ax2.set_xlabel('bin width %5.1f msec' % (1e3 * ibin_width))
    
    cm = plt.get_cmap('Set1')
    cNorm  = colors.Normalize(vmin=0, vmax=values[-1])
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cm)
#     ax1.set_axis_bgcolor('k')
#     ax2.set_axis_bgcolor('k')
    
    for i in range(8):
        colorVal = scalarMap.to_rgba(values[i])
        ax1.bar(ind+i*bar_width, duration_histo[i,:], bar_width, edgecolor='none', color=colorVal)

    for i in range(8):
        colorVal = scalarMap.to_rgba(values[i])
        ax2.bar(ind+i*bar_width, interval_histo[i,:], bar_width, edgecolor='none', color=colorVal)

    
    plt.show()
