﻿from H3DInterface import *
from H3DUtils import *

import sys
import os
from Queue import Queue, Empty
from inspect import getmembers, isfunction
import operator
from importlib import import_module
import traceback

def getBaseSFieldType ( field ):
  """ Returns the fundamental type of the field instance, e.g. SFInt32,
      ignoring any user defined subclasses.
  """

  type_names = filter ( lambda x: x[0] == field.type, sfield_types )
  if type_names:
    return globals()[type_names[0][1]]
  
  return None

def SGTimerCallback ( BaseField ):
  """ A field to schedule timed callbacks similar to H3DUtil.TimerCallback
      but supporting any type of time source (which allows the > operator) not
      only SFTime.
  """

  class SGTimerCallbackImpl( AutoUpdate( BaseField ) ):
    ## Constructor.
    def __init__( self ):
      AutoUpdate( BaseField ).__init__( self )
      ## The list of callbacks currently in use.
      self.callbacks = []

    ## Specialized update function to call callback functions when the time
    ## is right.
    def update( self, event ):
      t = event.getValue()
      cbs_to_remove = []
      for cb in self.callbacks:
        if t >= cb[0]:
          apply( cb[1], cb[2] )
          cbs_to_remove.append( cb )
      for cb in cbs_to_remove:
        self.callbacks.remove( cb )

      return event.getValue()

    ## Add a callback function. The function will be called at the specified
    ## time with the given arguments and then removed.
    ## \param time The time to run the function.
    ## \param func The function to call.
    ## \param args Tuple with the arguments to call.
    ## \return Handle to callback function.
    def addCallback( self, time, func, args ):
      cb = (time, func, args )
      self.callbacks.append( cb )
      return cb

    ## Remove a callback function before it has executed.
    ## \param cb The handle of the callback to remove.
    def removeCallback( self, cb ):
      try:
        self.callbacks.remove( cb )
      except:
        pass

  return SGTimerCallbackImpl()


def initTests(default_time_source = None, default_absolute_time = None, init_start_time = None):
  if testHelper is None:
    return False
  else:
    return testHelper.initTests(default_time_source, default_absolute_time, init_start_time)
        

timer_callbacks = {}
def scheduleMultiTimesourceCallback(timeSourceList, timeList, func, data):
  """ Schedule delayed callbacks using custom fields as the timer, but with multiple timer fields and different time values for each of them
  """

  def checkAllTimesources(timeSourceList, timeList, func, data):
    """
      A function that checks if all the time sources have passed the target time. If they have then it will call the callback function.
      The idea is that this gets triggered by every separate timer as they get to their target times.
      When the last one triggers the values of all the time sources should be greater than or equal to their respective target times so func finally gets called.

    """
    try:
      all_sources_passed = True
      for timepair in zip(timeSourceList, timeList):
        if timepair[0].getValue() < timepair[1]:
          all_sources_passed = False
      if all_sources_passed:
        func(*data)
        # remove our route out
    except:
      traceback.print_exc()

  try:
    # We create a separate timer for each time source, but all the timers are instructed to call checkAlltimesources with the full list of sources and times as parameters
    for timepair in zip(timeSourceList, timeList):
      if id(timepair[0]) not in timer_callbacks:
        timer = SGTimerCallback ( type(timepair[0]) )
        timer_callbacks[id(timepair[0])] = timer
      else:
        timer = timer_callbacks[id(timepair[0])]

      # Add out callback
      timer.addCallback ( timepair[1], checkAllTimesources, (timeSourceList, timeList, func, data) )
      # Re-route the timeSource to safely trigger the timer's update function so we can check if the timeSource is already past the target time
      timepair[0].unroute ( timer )
      timepair[0].route( timer )
  except:
    traceback.print_exc()



def scheduleCallback ( timeSource, time, func, data ):
  """ Schedule a delayed callback, using a custom field as the timer.
  """
  
  if not id(timeSource) in timer_callbacks:
    base_type = getBaseSFieldType( timeSource )
    if base_type is None:
      raise Exception( "Unsupported time source field type!" )
    timer = SGTimerCallback( base_type )
    timer_callbacks[id(timeSource)] = timer
  else:
    timer = timer_callbacks[id(timeSource)]
  
  # Add out callback
  timer.addCallback ( time, func, data )
  # Re-route the timeSource to safely trigger the timer's update function so we can check if the timeSource is already past the target time
  timeSource.unroute ( timer )
  timeSource.route( timer )

class TestHelper:
  def __init__( self, early_shutdown_file, output_file_prefix, output_filename_prefix, external_init, init_func):
    self.test_funcs = Queue()
    self.screenshot_queue = Queue()
    self.early_shutdown_file = early_shutdown_file
    self.output_file_prefix = output_file_prefix
    self.screenshot_counter = 0    
    self.output_filename_prefix = output_filename_prefix
    self.validation_file = output_file_prefix + "/validation.txt"
    self.last_func = None
    self.customPrintHelper = None
    self.external_init = external_init
    self.init_func = init_func
    self.init_done = False
    self.default_time_source = time
    self.default_absolute_time = False

    temp = open(self.validation_file, 'w')
    temp.flush()
    temp.close()

  def initTests(self, default_time_source, default_absolute_time, init_start_time):
    try:
      if not default_time_source is None:
        self.default_time_source = default_time_source
      if not default_absolute_time is None:
        self.default_absolute_time = default_absolute_time


      if self.external_init:
        if isinstance(self.default_time_source, list): # If time_source is a list of time sources then we need to calculate start times (based on absolute_time) and then use scheduleMultiTimesourceCallback instead of scheduleCallback
          calculated_start_times = []
          if self.default_absolute_time:
            calculated_start_times = init_start_time
          else:
            for i in range(len(self.default_time_source)):
              if not init_start_time is None:
                calculated_start_times.append(self.default_time_source[i].getValue()+init_start_time[i])
              else:
                calculated_start_times.append(self.default_time_source[i].getValue())
        else:
          calculated_start_times = None

        if isinstance(self.default_time_source, list):
          scheduleMultiTimesourceCallback( self.default_time_source, calculated_start_times, TestHelper.doTesting, (self,) )
        else:
          scheduleCallback ( self.default_time_source, (0 if self.default_absolute_time else self.default_time_source.getValue())+(StartTime if init_start_time is None else init_start_time), TestHelper.doTesting, (self,))
    
      return self.external_init
    except Exception as e:
      #write STEP_RAISED_EXCEPTION to the validation file
      try:
        traceback.print_exc()
        f = open(self.validation_file, 'a')
        f.write('initTests()\n')
        f.write("STEP_RAISED_EXCEPTION: \n")
        f.write(traceback.format_exc())
        f.write("STEP_RAISED_EXCEPTION_END\n")
        f.flush()
        f.close()
      except Exception as e: # And if we can't even write that we got an exception then we throw a fit so it absolutely won't be missed
        traceback.print_exc()
        shutdown_file = open( self.early_shutdown_file, 'w' )
        shutdown_file.write( "OK" )
        shutdown_file.flush()
        shutdown_file.close()
        throwQuitAPIException()

  def addTests(self, funclist):
    for func in funclist:
      self.test_funcs.put(func)

  def doTesting(self):
    if not self.init_done:
      if not self.init_func is None:
        try:
          self.init_func()
        except Exception as e:
          #write STEP_RAISED_EXCEPTION to the validation file
          try:
            traceback.print_exc()
            f = open(self.validation_file, 'a')
            f.write('testInitFunc\n')
            f.write("STEP_RAISED_EXCEPTION: \n")
            f.write(traceback.format_exc())
            f.write("STEP_RAISED_EXCEPTION_END\n")
            f.flush()
            f.close()
          except Exception as e: # And if we can't even write that we got an exception then we throw a fit so it absolutely won't be missed
            traceback.print_exc()
            shutdown_file = open( self.early_shutdown_file, 'w' )
            shutdown_file.write( "OK" )
            shutdown_file.flush()
            shutdown_file.close()
            throwQuitAPIException()
      self.init_done = True

    if not self.last_func is None:
      try:
        for validation in self.last_func.validation:
          validation['post'](self, validation, self.validation_file)
      except Exception as e:
        traceback.print_exc()
   
    try: # If there're more test functions queued up they'll be called, otherwise we shut down.
      func_data = self.test_funcs.get(False)
      self.last_step_name = func_data[0]
      is_test_step = False
      
      func = func_data[1]

      for validation in func.validation:
        if validation['type'] != 'none':
          is_test_step = True
      if is_test_step:
        try:
          f = open(self.validation_file, 'a')
          f.write(self.last_step_name + '\n')
          f.flush()
          f.close()
        except Exception as e:
          traceback.print_exc()

      self.last_func = func_data[1]
      start_time = None
      run_time = None
      time_source = self.default_time_source
      absolute_time = self.default_absolute_time
      for validation in func.validation:
        if 'start_time' in validation:
          if start_time is None or isinstance(start_time, list) or start_time < validation['start_time']:
            start_time = validation['start_time']
        if 'run_time' in validation:
          if run_time is None or isinstance(run_time, list) or run_time < validation['run_time']:
            run_time = validation['run_time']
        if 'time_source' in validation and not validation['time_source'] is None:
          time_source = validation['time_source']
        if 'absolute_time' in validation and not validation['absolute_time'] is None:
          absolute_time = validation['absolute_time']

      if isinstance(time_source, list): # If time_source is a list of time sources then we need to calculate start times (based on absolute_time) and then use scheduleMultiTimesourceCallback instead of scheduleCallback
        calculated_start_times = []
        if absolute_time:
          calculated_start_times = start_time
        else:
          for i in range(len(time_source)):
            if not start_time is None:
              calculated_start_times.append(time_source[i].getValue()+start_time[i])
      else:
        calculated_start_times = None

      if not(start_time is None):
        if isinstance(time_source, list):
          scheduleMultiTimesourceCallback( time_source, calculated_start_times, TestHelper.startFuncDelayed, (self, func, run_time, time_source) )
        else:
          scheduleCallback ( time_source, (0 if absolute_time else time_source.getValue())+start_time, TestHelper.startFuncDelayed, (self, func, run_time, time_source) )
      else:
        try:
          for validation in func.validation:
            validation['init'](self, validation, self.validation_file)
        except Exception as e:
          traceback.print_exc()
        try:
          func()
        except Exception as e:
          #write STEP_RAISED_EXCEPTION to the validation file
          try:
            traceback.print_exc()
            f = open(self.validation_file, 'a')
            f.write("STEP_RAISED_EXCEPTION: \n")
            f.write(traceback.format_exc())
            f.write("STEP_RAISED_EXCEPTION_END\n")
            f.flush()
            f.close()
          except Exception as e: # And if we can't even write that we got an exception then we throw a fit so it absolutely won't be missed
            traceback.print_exc()
            shutdown_file = open( self.early_shutdown_file, 'w' )
            shutdown_file.write( "OK" )
            shutdown_file.flush()
            shutdown_file.close()
            throwQuitAPIException()
            
            
            
        if isinstance(time_source, list):
          calculated_run_times = []
          for i in range(len(time_source)):
            if not run_time is None:
              if isinstance(run_time, list):
                calculated_run_times.append(time_source[i].getValue()+run_time[i])
              else:
                calculated_run_times.append(time_source[i].getValue()+run_time)
            else:
              calculated_run_times.append(time_source[i].getValue()+1)
          scheduleMultiTimesourceCallback( time_source, calculated_run_times, TestHelper.doTesting, (self,) )
        elif(not (run_time is None)):
          scheduleCallback ( time_source, time_source.getValue()+run_time, TestHelper.doTesting, (self,) )
        else:
          scheduleCallback ( time_source, time_source.getValue()+1, TestHelper.doTesting, (self,) )

      return
    except Exception as e:
      if not isinstance(e, Empty): # If we got an Empty exception then we simply ran out of test steps and should shut down. If we got something else then it was an error that should be reported.
        traceback.print_exc()
        try:
          f = open(self.validation_file, 'a')
          f.write('doTesting(in the boilerplate)\n')
          f.write("STEP_RAISED_EXCEPTION: \n")
          f.write(traceback.format_exc())
          f.write("STEP_RAISED_EXCEPTION_END\n")
          f.flush()
          f.close()
        except:
          pass
      shutdown_file = open( self.early_shutdown_file, 'w' )
      shutdown_file.write( "OK" )
      shutdown_file.flush()
      shutdown_file.close()
      throwQuitAPIException()

  def startFuncDelayed(self, func, run_time, time_source):
      try:
        for validation in func.validation:
          validation['init'](self, validation, self.validation_file)
      except Exception as e:
        traceback.print_exc()
      try:
        func()
      except Exception as e:
        #write STEP_RAISED_EXCEPTION to the validation file
        try:
          traceback.print_exc()
          f = open(self.validation_file, 'a')
          f.write("STEP_RAISED_EXCEPTION: \n")
          f.write(traceback.format_exc())
          f.write("STEP_RAISED_EXCEPTION_END\n")
          f.flush()
          f.close()
        except Exception as e: # And if we can't even write that we got an exception then we throw a fit so it absolutely won't be missed
          traceback.print_exc()
          shutdown_file = open( self.early_shutdown_file, 'w' )
          shutdown_file.write( "OK" )
          shutdown_file.flush()
          shutdown_file.close()
          throwQuitAPIException()

      try:
        if isinstance(time_source, list):
          calculated_run_times = []
          for i in range(len(time_source)):
            if not run_time is None:
              if isinstance(run_time, list):
                calculated_run_times.append(time_source[i].getValue()+run_time[i])
              else:
                calculated_run_times.append(time_source[i].getValue()+run_time)
            else:
              calculated_run_times.append(time_source[i].getValue()+0.5)

          scheduleMultiTimesourceCallback( time_source, calculated_run_times, TestHelper.doTesting, (self,) )
        elif(not (run_time is None)):
          scheduleCallback ( time_source, time_source.getValue()+run_time, TestHelper.doTesting, (self,))
        else:
          scheduleCallback ( time_source, time_source.getValue()+0.5, TestHelper.doTesting, (self,))
      except Exception as e:
        #write STEP_RAISED_EXCEPTION to the validation file
        try:
          traceback.print_exc()
          f = open(self.validation_file, 'a')
          f.write("STEP_RAISED_EXCEPTION: \n")
          f.write(traceback.format_exc())
          f.write("STEP_RAISED_EXCEPTION_END\n")
          f.flush()
          f.close()
        except Exception as e: # And if we can't even write that we got an exception then we throw a fit so it absolutely won't be missed
          traceback.print_exc()
          shutdown_file = open( self.early_shutdown_file, 'w' )
          shutdown_file.write( "OK" )
          shutdown_file.flush()
          shutdown_file.close()
          throwQuitAPIException()

  def printCustom(self, value):
    if self.customPrintHelper is None:
      print "Error: Test script called printCustom from a function that does not have the @custom decorator"
    else:
      self.customPrintHelper(self, value)


TestCaseScriptFolder = getNamedNode('TestCaseScriptFolder').getField('value').getValueAsString().replace('"', '')
TestCaseDefFolder = getNamedNode('TestCaseDefFolder').getField('value').getValueAsString().replace('"', '')
TestBaseFolder = getNamedNode('TestBaseFolder').getField('value').getValueAsString().replace('"', '')
sys.path.append(TestBaseFolder) # This is so we can properly import from UnitTestUtil.py
TestCaseScriptFilename = getNamedNode('TestCaseScriptFilename').getField('value').getValueAsString().replace('"', '')
sys.path.append(TestCaseScriptFolder)
TestCaseName = getNamedNode('TestCaseName').getField('value').getValueAsString().replace('"', '')
StartTime = getNamedNode('StartTime').getField('value').getValue()[0]
res = import_module(TestCaseScriptFilename)
res.__scriptnode__ = globals()['__scriptnode__']

def linenumber_of_member(m):
    try:
        return m[1].func_code.co_firstlineno
    except AttributeError:
        return -1

# import all the functions that have our validator decorators attached. We identify them by the presence of a validation array.
# the result is a list of tuples containing (function name, function address), so we sort by the latter to ensure the test functions will be executed in the same order as they appear in the file
testfunctions_list = []
init_func = None
external_init = False
init_time_source = time
init_absolute_time = False

for elem in getmembers(res):
  if isfunction(elem[1]):
   if hasattr(elem[1], "validation"):
     testfunctions_list.append(elem)
   elif elem[0] == "testInitFunc":
     init_func = elem[1]
  elif elem[0] == "external_init":
    external_init = elem[1]
  elif elem[0] == "test_init_time_source":
    init_time_source = elem[1]
  elif elem[0] == "test_init_absolute_time":
    init_absolute_time = elem[1]

testfunctions_list.sort(key=linenumber_of_member)

testHelper = TestHelper(TestBaseFolder+"/test_complete", os.path.abspath(os.path.join(TestCaseDefFolder, "output").replace("\\", '/')), TestCaseName + '_', external_init, init_func)
testHelper.addTests(testfunctions_list)
res.printCustom = testHelper.printCustom
res.TestCaseDefFolder = TestCaseDefFolder

if (external_init is None) or not external_init:
  scheduleCallback ( init_time_source, (0 if init_absolute_time else init_time_source.getValue())+StartTime, TestHelper.doTesting, (testHelper,))

import UnitTestUtil

def traverseSG():
  # Update the graphics frame counter each traverse
  # Tests may use this field as a time source to trigger events
  UnitTestUtil.graphics_frame_counter.setValue( UnitTestUtil.graphics_frame_counter.getValue() + 1 )