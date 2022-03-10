# -*- coding: utf-8 -*-
"""
Created on Tue Dec 15 13:51:48 2015

@author: Andrew Erskine
"""

# region [Import]
from PyDAQmx import *
from ctypes import *
import daqface.Utils as Util
import numpy
import matplotlib.pyplot as plt
import time


# region [DigitalTasks]


class DigitalInput(Task):
    def __init__(self, device, channels, samprate, secs, clock=''):
        Task.__init__(self)
        self.CreateDIChan(device, "", DAQmx_Val_ChanPerLine)

        self.read = int32()
        self.channels = channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.digitalData = numpy.ones((channels, self.totalLength), dtype=numpy.uint32)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.WaitUntilTaskDone(-1)
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        print('Starting digital input')
        self.StartTask()
        self.ReadDigitalU32(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.digitalData,
                            self.totalLength * self.channels, byref(self.read), None)

    def DoneCallback(self, status):
        print(status)
        self.StopTask()
        self.ClearTask()
        return 0


class TriggeredDigitalInput(Task):
    def __init__(self, device, channels, samprate, secs, trigger_source, clock=''):
        Task.__init__(self)
        self.CreateDIChan(device, "", DAQmx_Val_ChanPerLine)

        self.read = int32()
        self.channels = channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.digitalData = numpy.zeros((channels, self.totalLength), dtype=numpy.uint32)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.WaitUntilTaskDone(-1)
        self.CfgDigEdgeStartTrig(trigger_source, DAQmx_Val_Rising)
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        self.StartTask()
        self.ReadDigitalU32(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.digitalData,
                            self.totalLength * self.channels, byref(self.read), None)

    def DoneCallback(self, status):
        print(status.value)
        self.StopTask()
        self.ClearTask()
        return 0


class DigitalOut(Task):
    def __init__(self, device, samprate, secs, write, clock=''):
        Task.__init__(self)
        self.CreateDOChan(device, "", DAQmx_Val_ChanPerLine)

        self.sampsPerChanWritten = int32()
        self.totalLength = samprate * secs
        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))

        self.AutoRegisterDoneEvent(0)

        self.write = Util.binary_to_digital_map(write)

    def DoTask(self):
        print ('Starting digital output')
        self.WriteDigitalU32(self.write.shape[1], 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)

        self.StartTask()

    def DoneCallback(self, status):
        print(status)
        self.StopTask()
        self.ClearTask()
        return 0


class ThreadSafeDigitalOut:
    def __init__(self, device, samprate, secs, write, clock=''):
        self.do_handle = TaskHandle(0)

        DAQmxCreateTask("", byref(self.do_handle))

        DAQmxCreateDOChan(self.do_handle, device, '', DAQmx_Val_ChanPerLine)

        self.sampsPerChanWritten = int32()
        self.write = Util.binary_to_digital_map(write)

        self.totalLength = numpy.uint64(samprate * secs)

        DAQmxCfgSampClkTiming(self.do_handle, clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxWriteDigitalU32(self.do_handle, self.write.shape[1], 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)

        DAQmxStartTask(self.do_handle)
        DAQmxWaitUntilTaskDone(self.do_handle, DAQmx_Val_WaitInfinitely)

        self.ClearTasks()

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.do_handle)

        DAQmxClearTask(self.do_handle)


# region [AnalogTasks]


class AnalogInput(Task):
    def __init__(self, device, channels, samprate, secs, clock=''):
        Task.__init__(self)
        self.CreateAIVoltageChan(device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, None)

        self.read = int32()
        self.channels = channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.analogRead = numpy.zeros((channels, self.totalLength), dtype=numpy.float64)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.WaitUntilTaskDone(-1)
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        self.StartTask()
        self.ReadAnalogF64(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogRead,
                           self.totalLength * self.channels, byref(self.read), None)

    def DoneCallback(self, status):
        self.StopTask()
        self.ClearTask()
        return 0


class ThreadSafeAnalogInput:
    def __init__(self, ai_device, channels, samp_rate, secs, clock=''):
        self.ai_handle = TaskHandle(0)

        DAQmxCreateTask("", byref(self.ai_handle))

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)

        self.ai_read = int32()
        self.ai_channels = channels
        self.totalLength = numpy.uint64(samp_rate * secs)
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxStartTask(self.ai_handle)
        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           numpy.uint32(self.ai_channels*self.totalLength), byref(self.ai_read), None)
        self.ClearTasks()
        return self.analogData

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.ai_handle)
        DAQmxClearTask(self.ai_handle)


class TriggeredAnalogInput(Task):
    def __init__(self, device, channels, samprate, secs, trigger_source, clock=''):
        Task.__init__(self)
        self.CreateAIVoltageChan(device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, None)

        self.read = int32()
        self.channels = channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.analogRead = numpy.zeros((channels, self.totalLength), dtype=numpy.float64)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.WaitUntilTaskDone(-1)
        self.CfgDigEdgeStartTrig(trigger_source, DAQmx_Val_Rising)
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        self.StartTask()
        self.ReadAnalogF64(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogRead,
                           self.totalLength * self.channels, byref(self.read), None)

    def DoneCallback(self, status):
        print(status)
        self.StopTask()
        self.ClearTask()
        return 0


class AnalogOutput(Task):
    def __init__(self, device, samprate, secs, write, clock=''):
        Task.__init__(self)
        self.CreateAOVoltageChan(device, "", -10.0, 10.0, DAQmx_Val_Volts, None)

        self.sampsPerChanWritten = int32()
        self.write = write
        self.totalLength = numpy.uint32(samprate * secs)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        self.WriteAnalogF64(self.write.shape[1], 0, -1, DAQmx_Val_GroupByChannel,
                            self.write, byref(self.sampsPerChanWritten), None)
        self.StartTask()

    def DoneCallback(self, status):
        print(status)
        self.StopTask()
        self.ClearTask()
        return 0


# region [MultiTasks]
class DoTriggeredCoTask:
    def __init__(self, do_device, co_device, samp_rate, secs, write, trigger_source, controlled_carrier=False):
        self.do_handle = TaskHandle(0)
        self.co_handle = TaskHandle(1)
        self.do_device = do_device


        DAQmxCreateTask('', byref(self.do_handle))
        DAQmxCreateTask('', byref(self.co_handle))

        DAQmxCreateCOPulseChanFreq(self.co_handle, '/cDAQ1/Ctr0', '', DAQmx_Val_Hz, DAQmx_Val_Low, 0.0, samp_rate, 0.5)
        DAQmxCreateDOChan(self.do_handle, do_device, '', DAQmx_Val_ChanForAllLines)
        

        DAQmxCfgDigEdgeStartTrig(self.co_handle, trigger_source, DAQmx_Val_Rising)
        self.totalLength = numpy.uint64(samp_rate * secs)
        self.secs = secs
        self.sampsPerChanWritten = int32()
        self.write = Util.binary_to_digital_map(write)
        self.sampsPerChan = self.write.shape[1]
        self.chans = self.write.shape[0]
        self.write = numpy.sum(self.write, axis=0)
        if controlled_carrier:
            self.write += 2**(self.chans+1)
        DAQmxCfgImplicitTiming(self.co_handle, DAQmx_Val_FiniteSamps, self.totalLength)
        DAQmxCfgSampClkTiming(self.do_handle, '/cDAQ1/Ctr0InternalOutput', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))
        
    def DoTask(self):

        DAQmxWriteDigitalU32(self.do_handle, self.sampsPerChan, 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)


        DAQmxStartTask(self.do_handle)
        DAQmxStartTask(self.co_handle)
  
        DAQmxWaitUntilTaskDone(self.co_handle, 100)
        DAQmxWaitUntilTaskDone(self.do_handle, 100)

        self.ClearTasks()

    def ClearTasks(self):
        time.sleep(0.05)

        DAQmxStopTask(self.do_handle)
        DAQmxStopTask(self.co_handle)
 
        DAQmxClearTask(self.do_handle)
        DAQmxClearTask(self.co_handle)
        #closeValves(self.do_device)

class DoCoTask:
    def __init__(self, do_device, co_device, samp_rate, secs, write, controlled_carrier=False):
        self.do_handle = TaskHandle(0)
        self.co_handle = TaskHandle(1)
        self.do_device = do_device

        DAQmxCreateTask('', byref(self.do_handle))
        DAQmxCreateTask('', byref(self.co_handle))

        DAQmxCreateCOPulseChanFreq(self.co_handle, 'cDAQ1/Ctr0', '', DAQmx_Val_Hz, DAQmx_Val_Low, 0.0, samp_rate, 0.5)  ## Creates a channel to generate digital pulses
        DAQmxCreateDOChan(self.do_handle, do_device, '', DAQmx_Val_ChanForAllLines) 

        self.totalLength = numpy.uint64(samp_rate * secs)
        self.secs = secs
        self.sampsPerChanWritten = int32()
        self.write = Util.binary_to_digital_map(write)
        self.sampsPerChan = self.write.shape[1]
        self.chans = self.write.shape[0]
        self.write = numpy.sum(self.write, axis=0)
        # if controlled_carrier:
        #     self.write += 2**(self.chans+1)

        DAQmxCfgImplicitTiming(self.co_handle, DAQmx_Val_FiniteSamps, self.totalLength)
        DAQmxCfgSampClkTiming(self.do_handle, '/cDAQ1/Ctr0InternalOutput', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxWriteDigitalU32(self.do_handle, self.sampsPerChan, 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)

        DAQmxStartTask(self.do_handle)
        DAQmxStartTask(self.co_handle)
        DAQmxWaitUntilTaskDone(self.co_handle, 100)
        DAQmxWaitUntilTaskDone(self.do_handle, 100)
        self.ClearTasks()

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.do_handle)
        DAQmxStopTask(self.co_handle)

        DAQmxClearTask(self.do_handle)
        DAQmxClearTask(self.co_handle)
        #closeValves(self.do_device)

class DoAiMultiTask:
    def __init__(self, ai_device, ai_channels, do_device, samp_rate, secs, write, sync_clock):
        self.ai_handle = TaskHandle(0)
        self.do_handle = TaskHandle(1)

        DAQmxCreateTask('', byref(self.ai_handle))
        DAQmxCreateTask('', byref(self.do_handle))

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, '', DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)
        DAQmxCreateDOChan(self.do_handle, do_device, '', DAQmx_Val_ChanForAllLines)

        self.ai_read = int32()
        self.ai_channels = ai_channels
        self.sampsPerChanWritten = int32()

        self.write = Util.binary_to_digital_map(write)
        self.sampsPerChan = self.write.shape[1]
        self.write = numpy.sum(self.write, axis=0)

        self.totalLength = numpy.uint64(samp_rate * secs)
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))
        DAQmxCfgSampClkTiming(self.do_handle, sync_clock, samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxWriteDigitalU32(self.do_handle, self.sampsPerChan, 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)

        DAQmxStartTask(self.do_handle)
        DAQmxStartTask(self.ai_handle)

        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           numpy.uint32(self.ai_channels*self.totalLength), byref(self.ai_read), None)

        self.ClearTasks()
        return self.analogData

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.do_handle)
        DAQmxStopTask(self.ai_handle)

        DAQmxClearTask(self.do_handle)
        DAQmxClearTask(self.ai_handle)


class DoAiTriggeredMultiTask:
    def __init__(self, ai_device, ai_channels, do_device, samp_rate, secs, write, sync_clock, trigger_source):
        self.ai_handle = TaskHandle(0)
        self.do_handle = TaskHandle(1)

        DAQmxCreateTask('', byref(self.ai_handle))
        DAQmxCreateTask('', byref(self.do_handle))

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, '', DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)
        DAQmxCreateDOChan(self.do_handle, do_device, '', DAQmx_Val_ChanForAllLines)
        # DAQmxCfgAnlgEdgeStartTrig(self.ai_handle, trigger_source, DAQmx_Val_RisingSlope, 4.0)
        DAQmxCfgDigEdgeStartTrig(self.ai_handle, trigger_source, DAQmx_Val_Rising)

        self.ai_read = int32()
        self.ai_channels = ai_channels
        self.sampsPerChanWritten = int32()

        self.write = Util.binary_to_digital_map(write)
        self.sampsPerChan = self.write.shape[1]
        self.write = numpy.sum(self.write, axis=0)

        self.totalLength = numpy.uint64(samp_rate * secs)
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))
        DAQmxCfgSampClkTiming(self.do_handle, sync_clock, samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):

        DAQmxWriteDigitalU32(self.do_handle, self.sampsPerChan, 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)

        DAQmxStartTask(self.do_handle)
        DAQmxStartTask(self.ai_handle)

        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           numpy.uint32(self.ai_channels*self.totalLength), byref(self.ai_read), None)

        self.ClearTasks()
        return self.analogData

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.do_handle)
        DAQmxStopTask(self.ai_handle)

        DAQmxClearTask(self.do_handle)
        DAQmxClearTask(self.ai_handle)


class AoAiMultiTask:
    def __init__(self, ai_device, ai_channels, ao_device, samprate, secs, write, sync_clock):
        self.ai_handle = TaskHandle(0)
        self.ao_handle = TaskHandle(1)

        DAQmxCreateTask("", byref(self.ai_handle))
        DAQmxCreateTask("", byref(self.ao_handle))

        self.sampsPerChanWritten = int32()
        self.write = write
        self.totalLength = numpy.uint32(samprate * secs)

        self.ai_read = int32()
        self.ai_channels = ai_channels
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts,
                                 None)
        DAQmxCreateAOVoltageChan(self.ao_handle, ao_device, "", -10.0, 10.0, DAQmx_Val_Volts, None)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))
        DAQmxCfgSampClkTiming(self.ao_handle, sync_clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxWriteAnalogF64(self.ao_handle, self.write.shape[1], 0, -1, DAQmx_Val_GroupByChannel,
                            self.write, byref(self.sampsPerChanWritten), None)

        DAQmxStartTask(self.ao_handle)
        DAQmxStartTask(self.ai_handle)

        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           numpy.uint32(self.ai_channels*self.totalLength), byref(self.ai_read), None)

        self.ClearTasks()
        return self.analogData

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.ao_handle)
        DAQmxStopTask(self.ai_handle)

        DAQmxClearTask(self.ao_handle)
        DAQmxClearTask(self.ai_handle)



class MultiTask:
    def __init__(self, ai_device, ai_channels, di_device, di_channels, do_device, samprate, secs, write, sync_clock):
        self.ai_handle = TaskHandle(0)
        self.di_handle = TaskHandle(1)
        self.do_handle = TaskHandle(2)

        DAQmxCreateTask("", byref(self.ai_handle))
        DAQmxCreateTask("", byref(self.di_handle))
        DAQmxCreateTask("", byref(self.do_handle))

        # NOTE - Cfg_Default values may differ for different DAQ hardware
        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts,
                                 None)
        DAQmxCreateDIChan(self.di_handle, di_device, "", DAQmx_Val_ChanPerLine)

        self.ai_read = int32()
        self.di_read = int32()
        self.ai_channels = ai_channels
        self.di_channels = di_channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)
        self.digitalData = numpy.ones((self.di_channels, self.totalLength), dtype=numpy.uint32)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))
        DAQmxCfgSampClkTiming(self.di_handle, sync_clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxStartTask(self.di_handle)
        DAQmxStartTask(self.ai_handle)

        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           self.totalLength * self.ai_channels, byref(self.ai_read), None)
        DAQmxReadDigitalU32(self.di_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.digitalData,
                            self.totalLength * self.di_channels, byref(self.di_read), None)


def closeValves(do_device):
    do_handle = TaskHandle(0)
    co_handle = TaskHandle(1)
    DAQmxCreateTask('', byref(do_handle))
    DAQmxCreateTask('', byref(co_handle))

    DAQmxCreateCOPulseChanFreq(co_handle, 'cDAQ1/Ctr0', '', DAQmx_Val_Hz, DAQmx_Val_Low, 0.0, 20000, 0.5)
    DAQmxCreateDOChan(do_handle, do_device, '', DAQmx_Val_ChanForAllLines)
    
    DAQmxCfgImplicitTiming(co_handle, DAQmx_Val_FiniteSamps, 100)
    DAQmxCfgSampClkTiming(do_handle, '/cDAQ1/Ctr0InternalOutput', 20000, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(100))
    DAQmxWriteDigitalU32(do_handle, 100, 0, -1, DAQmx_Val_GroupByChannel, numpy.zeros(100, dtype=numpy.uint32),
                            byref(int32()), None)

    DAQmxStartTask(do_handle)
    DAQmxStartTask(co_handle)
    # DAQmxWaitUntilTaskDone(co_handle, 100)
    # DAQmxWaitUntilTaskDone(do_handle, 100)

    time.sleep(0.05)
    DAQmxStopTask(co_handle)
    DAQmxClearTask(co_handle)

    DAQmxStopTask(do_handle)
    DAQmxClearTask(do_handle)


## TODO generalise for other devices, currently needs to be the same one as the valves
# def carrierControlTask(do_device, samp_rate, secs):
#     do_handle = TaskHandle(2)
#     co_handle = TaskHandle(3)
#     DAQmxCreateTask('', byref(do_handle))
#     DAQmxCreateTask('', byref(co_handle))
#     DAQmxCreateCOPulseChanFreq(co_handle, 'cDAQ1/Ctr0', '', DAQmx_Val_Hz, DAQmx_Val_Low, 0.0, samp_rate, 0.5)  ## Creates a channel to generate digital pulses
#     DAQmxCreateDOChan(do_handle, do_device, '', DAQmx_Val_ChanForAllLines) 
#     totallength = numpy.uint64(samp_rate * secs)
#     secs = secs
#     sampsPerChanWritten = int32()
#     chan_index = int(do_device[-1])
#     write = np.ones(totallength)*(2**chan_index)
#     write = write.astype(np.uint32)
#     print(write)
#     DAQmxCfgImplicitTiming(co_handle, DAQmx_Val_FiniteSamps, totallength)
#     DAQmxCfgSampClkTiming(do_handle, '/cDAQ1/Ctr0InternalOutput', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
#                             numpy.uint64(totallength))
#     DAQmxWriteDigitalU32(do_handle, len(write), 0, -1, DAQmx_Val_GroupByChannel, write,
#                             byref(sampsPerChanWritten), None)
#     DAQmxStartTask(do_handle)
#     DAQmxStartTask(co_handle)
#     DAQmxWaitUntilTaskDone(co_handle, 100)
#     DAQmxWaitUntilTaskDone(do_handle, 100)
#     # do_handle = TaskHandle(2)
#     # DAQmxCreateTask('', byref(do_handle))
#     # DAQmxCreateDOChan(do_handle, do_device, '', DAQmx_Val_ChanForAllLines) 
#     # totallength = numpy.uint64(samp_rate * secs)
#     # secs = secs
#     # sampsPerChanWritten = int32()
#     # write = np.ones(totallength)*512
#     # write = write.astype(np.uint32)
#     # print(write)
#     # DAQmxCfgSampClkTiming(do_handle, '/cDAQ1/Ctr0InternalOutput', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
#     #                         numpy.uint64(totallength))
#     # DAQmxWriteDigitalU32(do_handle, totallength, 0, -1, DAQmx_Val_GroupByChannel, write,
#     #                          byref(sampsPerChanWritten), None)
#     # DAQmxStartTask(do_handle)
#     # DAQmxWaitUntilTaskDone(do_handle, 10)
#     DAQmxStopTask(co_handle)
#     DAQmxClearTask(co_handle)
#     DAQmxStopTask(do_handle)
#     DAQmxClearTask(do_handle)


# carrierControlTask(do_device, 20000, 2)
# TODO TESTING #
# region DoAiMultiTaskTest
# a = DoAiMultiTask('cDAQ1Mod3/ai0', 1, 'cDAQ1Mod1/port0/line0', 1000.0, 1.0, numpy.zeros((2, 1000)),
#                   '/cDAQ1/ai/SampleClock')
# analog = a.DoTask()
#
# plt.plot(analog[0])
# plt.show()
# endregion

# region simple digital test
# DigitalOutput test
# a = DigitalOut('cDAQ1Mod1/port0/line0:1', 1, 1000, numpy.zeros((2, 1000)), clock='')
# a.DoTask()

# DigitalInput test
# a = DigitalInput('cDAQ1Mod2/port0/line0', 1, 1000, 1)
# a.DoTask()
# endregion

# MultiTask test
# a = MultiTask('cDAQ1Mod3/ai0', 1, 'cDAQ1Mod2/port0/line0', 1, 'cDAQ1Mod1/port0/line0', 1000, 2, numpy.zeros((1, 2000),
#               dtype=numpy.uint32), '/cDAQ1/ai/SampleClock')
#
# a.DoTask()
#
# plt.plot(a.digitalData[0])
# plt.show()

# AnalogInput
# a = AnalogInput('cDAQ1Mod3/ai0', 1, 1000, 1)
# a.DoTask()
#
# plt.plot(a.analogRead[0])
# plt.show()
