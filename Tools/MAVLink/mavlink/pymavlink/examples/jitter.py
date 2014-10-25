'''
Created on Oct 24, 2014

@author: markw
'''
from __builtin__ import file
import numpy as np

def update_dcmState(dcmState, dcmVals, i):
        # maintain dcmState
        if (dcmState == 'Active'):
            if dcmVals[i] == 0:
                dcmState = 'Nactive'
        elif (dcmState == 'Nactive'):
            if dcmVals[i] == 1:
                dcmState = 'Active'

if __name__ == '__main__':
    file = open('/home/markw/Desktop/autopilots (dropbox)/AUAV3/jitter/mpu6K_dcmUpdate10s.csv', 'ro')
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
    # specific case for MPU6K analysis
    intVals = dvals[0,:]
    dcmVals = dvals[1,:]
    
    # for each 1->0 transition of intVals, calculate time interval
    nInterrupts = nRecs - 1 - np.count_nonzero(dvals[0,:])
    int_interval = np.zeros(nInterrupts)
    dcm_latency = np.zeros(nInterrupts)
    dcm_update_duration = np.zeros(nInterrupts)
    DCM_LE_time = 0

    intStates = ['unknown', 'Active', 'Nactive']
    dcmStates = ['unknown', 'Active', 'Nactive']
   
    intState = 'unknown'
    dcmState = 'Nactive'

    count = 0
    for ivalid in range(nInterrupts):
        # init by looking for the nth interrupt
        if intVals[ivalid] == 0:   # leading edge of interrupt
            count += 1
            if (count > 2):
                intState = 'Active'
                lastIntLE_time = tvals[ivalid] # nth interrupt
                break
        
    curI = 0
    updateDone = False
    for i in range(ivalid+1, nRecs):
        if (intState == 'Active'):
            if intVals[i] == 1:   # trailing edge of interrupt
                intState = 'Nactive'
                # verify dcm state Nactive
                if (dcmState == 'Active') or (dcmVals[i] == 1):
                    print("error: dcmState Active at trailing edge of interrupt")
        elif (intState == 'Nactive'):
            if intVals[i] == 0:   # leading edge of interrupt
                # record interval from last leading edge
                intState = 'Active'
                if updateDone != True:
                    print("error: DCM update not complete at leading edge of interrupt")
                updateDone = False
                int_interval[curI] = tvals[i] - lastIntLE_time
                lastIntLE_time = tvals[i]
                curI += 1

            # between interrupts, monitor dcmState
            if (dcmState == 'Active'):
                if dcmVals[i] == 0:
                    dcmState = 'Nactive'
                    DCM_TE_time = tvals[i]
                    dcm_update_duration[curI] = DCM_TE_time - DCM_LE_time  
                    updateDone = True                  
            elif (dcmState == 'Nactive'):
                # leading edge, measure latency
                if dcmVals[i] == 1:
                    dcmState = 'Active'
                    DCM_LE_time = tvals[i]
                    dcm_latency[curI] = DCM_LE_time - lastIntLE_time
    
                
    # record number of interrupts used
    nInterrupts = curI
    
    print("number of interrupts sampled: %d" % nInterrupts)
    
    meanIntv = np.mean(int_interval[0:nInterrupts])
    stdev = np.std(int_interval[0:nInterrupts])
    pMax = np.max(int_interval[0:nInterrupts])
    pMin = np.min(int_interval[0:nInterrupts])
    print("MPU6000 interrupt interval mean: %g, min: %g, max: %g, stdev: %g" % (meanIntv, pMin, pMax, stdev))
    print("MPU6000 interrupt interval max jitter: %g" % (pMax - pMin))
    
    meanIntv = np.mean(dcm_latency[0:nInterrupts])
    stdev = np.std(dcm_latency[0:nInterrupts])
    pMax = np.max(dcm_latency[0:nInterrupts])
    pMin = np.min(dcm_latency[0:nInterrupts])
    print("DCM interrupt latency mean: %g, min: %g, max: %g, stdev: %g" % (meanIntv, pMin, pMax, stdev))
    print("DCM interrupt latency max jitter: %g" % (pMax - pMin))
        
    meanIntv = np.mean(dcm_update_duration[0:nInterrupts])
    stdev = np.std(dcm_update_duration[0:nInterrupts])
    pMax = np.max(dcm_update_duration[0:nInterrupts])
    pMin = np.min(dcm_update_duration[0:nInterrupts])
    print("DCM update duration mean: %g, min: %g, max: %g, stdev: %g" % (meanIntv, pMin, pMax, stdev))
    print("DCM update duration max jitter: %g" % (pMax - pMin))

