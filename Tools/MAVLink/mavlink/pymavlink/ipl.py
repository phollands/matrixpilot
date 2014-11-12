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
    ofile = open('./ipldata/iplvals.csv', 'w')
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
    iplVals = np.zeros(nRecs, dtype='uint8')
    for i in range(0, nRecs):
        iplVals[i] = dvals[0,i] + 2 * dvals[1,i] + 4 * dvals[2,i]
    
    ofile.write("time,duration,IPL\n")
        
    iplAccum = np.zeros(8)
    iplCount = np.zeros(8)
    mindt = np.zeros(8)
    maxdt = np.zeros(8)
    maxdt_time = np.zeros(8)
    last_start_time = np.zeros(8)    
    cur_ipl = iplVals[0]

    duration = tvals[-1] - tvals[0]   
    for i in range(0, 8):
        mindt[i] = duration
        maxdt[i] = 0
    
    # histograms
    nbins = 50
    bin_width = 6e-6
    duration_histo = np.zeros((8, nbins))
    
    ibin_width = 1e-3
    interval_histo = np.zeros((8, nbins))
    
    traceTime = []
    tracedt = []
    traceIPL = []

    # calculate time spent at each IPL
    for i in range(1, nRecs):
        dt = tvals[i] - tvals[i-1]
        lastIPL = iplVals[i-1]
        
        # ignore glitches due to non-atomic DIGn assignment
        if dt < 100e-9: 
            continue
        
        traceTime.append(tvals[i-1])
        tracedt.append(dt)
        traceIPL.append(lastIPL)
        ofile.write("%10.9f,%10.9f,%d\n" % (tvals[i-1], dt, lastIPL))

        if dt < mindt[lastIPL]: 
            mindt[lastIPL] = dt
            
        if dt > maxdt[lastIPL]: 
            maxdt[lastIPL] = dt
            maxdt_time[lastIPL] = tvals[i]
            
        iplAccum[lastIPL] += dt
        iplCount[lastIPL] += 1
        
        bin_num = dt / bin_width
        if bin_num >= nbins: bin_num = nbins-1
        duration_histo[lastIPL, bin_num] += 1
        
        if last_start_time[lastIPL] != 0:
            tint = tvals[i] - last_start_time[lastIPL]
            bin_num = tint / ibin_width
            if bin_num >= nbins: bin_num = nbins-1
            interval_histo[lastIPL, bin_num] += 1
        last_start_time[lastIPL] = tvals[i]
        
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
