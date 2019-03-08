# All decorators should add an array called validation to the function if it does not already have one. This array should contain one entry for every decorator that then contains any validation-specific data.
# The parameters start_time and run_time will, if set on the function, determine how long it waits before executing the function and how long it waits after executing the function respectively. It is optional for a decorator to add these.
# It should add an entry into that array containing at least the following:
# init(testHelper, validator, validation_output_file) a function that initiates any data the validator needs. This will be run before the test step function is called. validation_output_file is a file that can and should have any validation output appended to it.
# post(testHelper, validator, validation_output_file) a function that stores validation data. This will be run the next scene graph iteration after run_time has passed. If run_time is undefined then or 0 then it will be run the next iteration after the test function is called.

import os, sys
from collections import deque
from H3DInterface import *
from H3DUtils import *

def screenshot(start_time = None, run_time = None, time_source = None, absolute_time = None):
  def _screenshot(func):
    def init(testHelper, validator, validation_output_file):
      pass

    def post(testHelper, validator, validation_output_file):
      try:
        screenshot_name = validator['step_name']
        filename = os.path.abspath(os.path.join(testHelper.output_file_prefix, "renderings/", testHelper.output_filename_prefix + validator['step_name'] + '.png').replace('\\', '/'))
        takeScreenshot(filename)
    #    pp.pprint("saving to " + self.validation_file)
        f = open(validation_output_file, 'a')
        f.write('screenshot\n')
        f.write(filename + '\n')
        f.flush()
        f.close()
      except Exception as e:
        if str(e) != '':
          print(str(e))   
    
    if not hasattr(func, 'validation'):
      func.validation = []

    func.validation.append({'type': 'screenshot', 'step_name' : func.__name__, 'init' :  init, 'post' : post, 'start_time' : start_time, "run_time" : run_time, "time_source" : time_source, "absolute_time" : absolute_time})
    return func
  return _screenshot

def image(start_time = None, run_time = None, time_source = None, absolute_time = None, image_name = None, fuzz=5, threshold=20):
  def _image(func):
    def init(testHelper, validator, validation_output_file):
      pass

    def post(testHelper, validator, validation_output_file):
      try:
        image_name = validator['image_name']
        filename = os.path.abspath(os.path.join(testHelper.output_file_prefix, image_name).replace('\\', '/'))
    #    pp.pprint("saving to " + self.validation_file)
        f = open(validation_output_file, 'a')
        f.write('image\n')
        f.write(filename + '\n')
        f.write(str(fuzz) + ',' + str(threshold) + '\n')
        f.flush()
        f.close()
      except Exception as e:
        if str(e) != '':
          print(str(e))   
    
    if not hasattr(func, 'validation'):
      func.validation = []

    func.validation.append({'type': 'image', 'step_name' : func.__name__, 'init' :  init, 'post' : post, 'start_time' : start_time, "run_time" : run_time, "time_source" : time_source, "absolute_time" : absolute_time, 'image_name' : image_name, 'fuzz' : fuzz, 'threshold' : threshold})
    return func
  return _image



def performance(start_time = None, run_time = None, time_source = None, absolute_time = None):
  class FramerateCounter( AutoUpdate( SFFloat ) ):
    def __init__( self):
      AutoUpdate( SFFloat ).__init__(self)
      self.running = False
      self.fps_data = deque(maxlen=10)

    def update( self, event ):
      if self.running:
        # Convert FPS to time in ms
        self.fps_data.append( 1000.0/event.getValue() )
      return 0

    def start( self ):
      self.fps_data.clear()
      self.running = True
      scene = getCurrentScenes()[0]
      scene.frameRate.routeNoEvent( self )
      scene = None

    def stop( self ):
      self.running = False
      return self.fps_data

  def _performance(func):
    def init(testHelper, validator, validation_output_file):
      try:
        getCurrentScenes()[0].getField("profiledResult").getValue()[0]
        testHelper.performance_uses_profiling = True
      except:
        testHelper.performance_uses_profiling = False # This means there's no profiledResult so we have to fall back on getting the framerate field from the Scene node instead
        print "not using profiler"
        testHelper.framerate_counter = FramerateCounter()
        testHelper.framerate_counter.start()

    def post(testHelper, validator, validation_output_file):
      if testHelper.performance_uses_profiling:
        try:
          profiling_data = getCurrentScenes()[0].getField("profiledResult").getValue()[0]
        except:
          print "Error: getting profiledResult failed! This build of h3d might not have ENABLE_PROFILER set."
          return
      else: # if it uses a FramerateCounter instead of the profiler
        try:
          counter = testHelper.framerate_counter
          fps_data = counter.stop()
          avg_fps = sum(fps_data)/len(fps_data)
          min_fps = min(fps_data)
          max_fps = max(fps_data)
          mean_fps = fps_data[len(fps_data)/2]
          total_fps = max_fps
          profiling_data = ''' Timer: H3D_scene
 LEVEL      START       NUM         MIN        MAX       MEAN       DEV        TOTAL     PERCENT     ID
 0          0           0           0          0         0          0          0         0           TOTAL
 1          0           1           %f         %f        %f         %f         %f        100       .H3D_scene_loop(framerate fallback)
====END====
''' % (min_fps, max_fps, mean_fps, max_fps, total_fps)
        except Exception as e:
          print "Error: Problems obtaining framerate from framerate counter. This build of h3d also didn't have ENABLE_PROFILER set, which is recommended.\nError was: " + str(e)    
          return
      try:
        f = open(validation_output_file, 'a')
        f.write('performance_start\n')
        f.write(profiling_data + '\n')
        f.write('performance_end\n')
        f.flush()
        f.close()
      except Exception as e:
        print str(e)


    if not hasattr(func, 'validation'):
      func.validation = []
      
    if (not (start_time is None) and (not hasattr(func, 'start_time') or (func.start_time < start_time))):
      func.start_time = start_time

    if (not (run_time is None) and (not hasattr(func, 'run_time') or (func.run_time < run_time))):
      func.run_time = run_time
      
    func.validation.append({'type': 'performance', 'step_name' : func.__name__, 'init' :  init, 'post' : post, 'start_time' : start_time, "run_time" : run_time, "time_source" : time_source, "absolute_time" : absolute_time})
    return func
  return _performance


def console(start_time = None, time_source = None, absolute_time = None):
  def _console(func):
    def init(testHelper, validator, validation_output_file):
      print 'console_start_' + validator['step_name']

    def post(testHelper, validator, validation_output_file):
      f = open(validation_output_file, 'a')
      f.write('console\n')
      f.flush()
      f.close()
      print 'console_end_' + validator['step_name']

    if not hasattr(func, 'validation'):
      func.validation = []
      
    if (not (start_time is None) and (not hasattr(func, 'start_time') or (func.start_time < start_time))):
      func.start_time = start_time


      
    func.validation.append({'type': 'console', 'step_name' : func.__name__, 'init' :  init, 'post' : post, 'start_time' : start_time, "run_time" : None, "time_source" : time_source, "absolute_time" : absolute_time})
    return func
  return _console


def custom( start_time=None, time_source=None, absolute_time=None, run_time=None ):
  def _custom(func):
    def customPrintHelper(testHelper, value):
      testHelper.customValidator['custom_output'] += value + "\n"

    def init(testHelper, validator, validation_output_file):
      testHelper.customValidator = validator
      testHelper.customPrintHelper = customPrintHelper
      validator['custom_output'] = ''

    def post(testHelper, validator, validation_output_file):
      testHelper.customPrintHelper = None
      try:
        f = open(validation_output_file, 'a')
        f.write('custom_start\n')
        f.write(validator['custom_output']) 
        f.write('custom_end\n')
        f.flush()
        f.close()
        f = open(os.path.abspath(os.path.join(testHelper.output_file_prefix, "text/" + testHelper.output_filename_prefix + validator['step_name'] + "_custom.txt")), 'w')
        f.write(validator['custom_output'])
        f.flush()
        f.close()
      except Exception as e:
        print(str(e))


    if not hasattr(func, 'validation'):
      func.validation = []
      
    if (not (start_time is None) and (not hasattr(func, 'start_time') or (func.start_time < start_time))):
      func.start_time = start_time

      
    func.validation.append({'type': 'custom', 'step_name' : func.__name__, 'init' :  init, 'post' : post, 'start_time' : start_time, "run_time" : run_time, "time_source" : time_source, "absolute_time" : absolute_time, 'custom_output' : ""})
    return func
  return _custom


def none( start_time=None, time_source=None, absolute_time=None, run_time=0 ):
  def _none(func):
    def init(testHelper, validator, validation_output_file):
      pass

    def post(testHelper, validator, validation_output_file):
      pass

    if not hasattr(func, 'validation'):
      func.validation = []
      
    if (not (start_time is None) and (not hasattr(func, 'start_time') or (func.start_time < start_time))):
      func.start_time = start_time

    func.validation.append({'type': 'none', 'step_name' : func.__name__, 'init' :  init, 'post' : post, 'start_time' : start_time, "run_time" : run_time, "time_source" : time_source, "absolute_time" : absolute_time})
    return func
  return _none

# A counter for the number of graphics frames executed
#
# Tests may use this field as a time source to trigger events
graphics_frame_counter = SFInt32()
graphics_frame_counter.setValue( 0 )
