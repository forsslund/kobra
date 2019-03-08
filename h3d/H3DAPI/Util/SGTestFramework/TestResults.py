import os
from collections import namedtuple
import glob
import subprocess
import difflib
import math
import string


class TestResults ( object ):
  # Container class for all kinds of possible test results. Also processes and validates the results
  # The get functions are used to validate results and will return a tuple that should be added to the array that is passed to addStepResults

  StepResultTuple = namedtuple('StepResult', ['step_name', 'success', 'results']) # results is an array of results
  PerformanceDataTuple = namedtuple('PerformanceData', ['level', 'id', 'mean', 'percent'])
  PerformanceResultTuple = namedtuple("PerformanceResult", ['error_type', 'profiling_data_full', 'profiling_data_lines']) # profiling_data should be an array of PerformanceDataTuples
  RenderingResultTuple = namedtuple("RenderingResult", ['error_type', 'success', 'baseline_path','output_path', 'diff_path', 'diff_pixels', 'threshold'])
  ConsoleResultTuple = namedtuple("ConsoleResult", ['error_type', 'success', 'baseline_path', 'output', 'diff'])
  CustomResultTuple = namedtuple("CustomResult", ['error_type', 'success', 'baseline_path', 'output', 'diff'])
  ErrorResultTuple = namedtuple("ErrorResult", ['error_type', 'stdout', 'stderr'])

  def __init__ ( self ):
    self.filename= ""
    self.case_name= ""
    self.result_type= ""
    self.url= ""
    self.std_out= ""
    self.std_err= ""
    self.svn_url_x3d = ""
    self.svn_url_script = ""
    self.starts_ok= False
    self.terminates_ok= False
  
    self.step_list = []
    
    self.warnings= 0
    self.errors= 0
    self.skipped= False
    self.success = True

  def addStepResults(step_name, success, results):
    self.step_results.append(self.StepResult(step_name, success, results))


  def getPerformanceResult(self, profiling_data):
    def parseLine(profiling_line):
      # LEVEL      START       NUM         MIN        MAX       MEAN       DEV        TOTAL     PERCENT     ID
      # It's possible a line might not contain another entry of this, especially now that python script nodes might be printed in their entirety. So to be on the safe side we make the assumption that
      # the first word has to be a digit. Python scripts don't generally start their lines with a lone constant integer.
      try:
        result = dict()
        if line.strip().partition(' ')[0].isdigit(): 
          data = profiling_line.split()
          result['LEVEL'] = data[0]
          result['START'] = data[1]
          result['NUM'] = data[2]
          result['MIN'] = data[3]
          result['MAX'] = data[4]
          result['MEAN'] = data[5]
          result['DEV'] = data[6]
          result['TOTAL'] = data[7]
          result['PERCENT'] = data[8]
          result['ID'] = data[9]
          return result
        else:
          return None
      except Exception as e:
        print "Parsing profiling data line failed: " + str(e) + '\nLine was: ' + profiling_line
        return None

    found_h3d_timer = False
    passed_total = False
    result = []
    for line in profiling_data.splitlines():
      if not found_h3d_timer:
        if line.strip() == 'Timer: H3D_scene':
          found_h3d_timer = True
      else:
        if line.replace('=', '').strip() == 'END' and passed_total:
          break # There's an empty line after the last line of profiling data in the ===== MAIN THREAD ===== section of the output. We use that to determine when we're done parsing
        
        out_data = parseLine(line)
        if out_data is None:
          pass
        elif out_data['ID'] == "TOTAL" and not passed_total:
          passed_total = True
        elif passed_total:
          result.append(TestResults.PerformanceDataTuple(out_data['LEVEL'], out_data['ID'], out_data['MEAN'], out_data['PERCENT']))
    if len(result) > 0:
      return TestResults.PerformanceResultTuple("NO_ERROR", profiling_data, result)
    else:
      return TestResults.ErrorResultTuple("CASE_STDERR", "Performance test didn't output profiler data. The loading time might be longer than the test, make sure its starttime is long enough or that the test script waits for long enough.", "")


  def getRenderingResult(self, baseline_path, output_path, fuzz, threshold):
    valid, diff_path, error, error_type, diff_pixels = self._validate_rendering(baseline_path.replace('\\', '/'), output_path.replace('\\', '/'), fuzz, threshold)
    for line in error:
      print line
    return TestResults.RenderingResultTuple(error_type, valid, baseline_path, output_path, diff_path, diff_pixels, threshold), error
  
  def getCustomResult(self, baseline_path, output):
    if not os.path.exists(baseline_path):
      return TestResults.CustomResultTuple("NO_BASELINE", False, "", output, "")
    else:
      valid, diff, error, error_type = self._validate_text(baseline_path, output)
      for line in error:
        print line
      return TestResults.CustomResultTuple(error_type, valid, baseline_path, output, diff)

  def getConsoleResult(self, baseline_path, output):
    if not os.path.exists(baseline_path):
      return TestResults.CustomResultTuple("NO_BASELINE", False, "", output, "")
    else:
      valid, diff, error, error_type  = self._validate_text(baseline_path, output)
      for line in error:
        print line
      return TestResults.ConsoleResultTuple(error_type, valid, baseline_path, output, diff)

  def _validate_rendering(self, baseline_path='.', rendering_path='.', fuzz=2, threshold=20):
    """ Compare renderings """
    valid = False
    diff_path = ''
    diff_pixels = 0 # The number of pixels by which the image diffed from the 
    error = []

    if not os.path.exists(rendering_path):
      error_type = "NO_OUTPUT"
      return False, "", error, error_type, 0
    elif not os.path.exists(baseline_path):
      error_type = "NO_BASELINE"
      return False, "", error, error_type, 0 
    else:
      error_type = "NO_ERROR"
    
    # Comparison thresholds
    error_threshold= threshold  # Number of pixels that must differ in order to trigger a warning

    g= glob.glob(baseline_path)
    if g:
      baselinerendering= max(g, key= os.path.getmtime)
      try:
        process= subprocess.Popen(
          [ "magick", "compare",
            "-fuzz","%d%%"%fuzz,
            "-metric","AE",
            baseline_path,rendering_path, os.path.split(rendering_path)[0] + '\\diff_' + os.path.split(rendering_path)[1]], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      except Exception as E:
        error_type = "CASE_STDERR"
        error.append("WARNING: Image comparison not available! Probably Imagemagick is not installed, or not in the path, or one of the following files do not exist:\n" +
                     "baseline: %s\noutput:%s\n" % (baseline_path,rendering_path) +
                     "Or the following file could not be created:\n" +
                     "diff: %s" % (os.path.split(rendering_path)[0] + '/diff_' + os.path.split(rendering_path)[1]) + "\n" +
                     "Error was: " + str(E))
        process= None

      if process:
        (output, err) = process.communicate()
        exit_code = process.wait()

        metric= None
        try:
          metric= int(err)
        except:
          valid = False;
          if "image widths or heights differ" in str(err):
            error_type = "SIZE_MISMATCH"
          error.append('WARNING: Image comparison failed!\n\nDetails:\n%s\n%s' % (rendering_path, err) )
          return valid, "", error, error_type, diff_pixels

        if metric is None:
          valid = False
        else:
          diff_pixels = metric
          if metric > error_threshold:
            valid = False
            error_type = "CONTENT_MISMATCH"
            diff_path = ( (os.path.split(rendering_path)[0] + '/diff_' + os.path.split(rendering_path)[1]) )

          else:
            # Test passed, we can clean up the file right now
            valid = True
            try:
              os.remove ( diff_path )
            except:
              pass#print "WARNING: Could not remove " + diff_path

    else:
      pass#error.append("WARNING: No rendering baseline, skipping validation!")
    return valid, diff_path, error, error_type, diff_pixels

  def _validate_text(self, baseline_path='.', output=''):
    try:
      valid = True
      diff = ''
      error = []
      differ = difflib.Differ()
      f = open(baseline_path, 'r')    
      baseline = [line.replace('\n', '') for line in f.readlines()]  
      f.close()
      output_list = [out.replace('\n', '') for out in output]
      delta = differ.compare(baseline, output_list)
      try:
        while True:
          line = delta.next()
          if line[0] != ' ':
            valid = False
          diff += line + '\n'
      except StopIteration:
        if valid:
          error_type = "NO_ERROR"
        else:
          error_type = "CONTENT_MISMATCH"
        return valid, diff, error, error_type #This is where we end up if going through the entire diff worked (even if the test failed)
    except Exception as e:
        print str(e)
        error.append(str(e))
    return False, diff, error, "CASE_STDERR" # We only get here if we got some breaking exception


  def getNextLine(self, file, IgnoreNewline=True):
    line = file.readline()
    while line and (IgnoreNewline and (line.strip() == '')):
      line = file.readline()
    return line.strip()

  def parseValidationFile(self, testcase, file_path='validation.txt', baseline_folder='', text_output_folder='', fuzz=0, threshold=0):
    self.step_list = []
    if not os.path.exists(file_path): # if the validation file doesn't exist then it failed completely, generate an error about that
      self.step_list.append(self.StepResultTuple("", False, [self.ErrorResultTuple("CASE_NOT_STARTED", self.std_out + "\nTest didn't output validation data, it probably crashed.", self.std_err)]))
      self.success = False
    else: # If the validation file exists, parse it    
      f = open(file_path, 'r')
      step_name = self.getNextLine(f)
      while step_name != None:
        success = True
        line = self.getNextLine(f)
        if line == '':
          if step_name != '':
            self.step_list.append(self.StepResultTuple(step_name, False, [self.ErrorResultTuple("STEP_NOT_RUN", "Test never finished executing this step", self.std_err)]))
            self.success = False
          break;
        results = []
        while line != None:
          if line.startswith("STEP_RAISED_EXCEPTION"):
            line = self.getNextLine(f, IgnoreNewline = False)
            output = []
            while line != 'STEP_RAISED_EXCEPTION_END':
              output.append(line + '\n')
              line = self.getNextLine(f, IgnoreNewline = False)
            results.append(self.ErrorResultTuple("CASE_STDERR", "Python exception was raised in " + step_name, "".join(output)))
            success = False
          elif line == 'screenshot':
            # Screenshot format: One line that says screenshot, one line with output screenshot path
            screenshot_path = self.getNextLine(f)
            res, render_error = self.getRenderingResult(baseline_folder + '\\' + os.path.split(screenshot_path)[1], screenshot_path, fuzz, threshold)
            success = success and res.success
            results.append(res)
            if (len(render_error) > 0) and res.error_type != "SIZE_MISMATCH":
              results.append(self.ErrorResultTuple("CASE_STDERR", 'Error when rendering validation for ' + screenshot_path + ':' + "\n".join(render_error), ''))
          elif line == 'image':
            # Image format: One line that says image, one line with output image path, one line with fuzz and threshold, separated by a comma
            image_path = self.getNextLine(f)
            fuzz, threshold = self.getNextLine(f).split(',')
            res, render_error = self.getRenderingResult(baseline_folder + '\\' + testcase.name+"_"+step_name+".png", image_path, int(fuzz), int(threshold))
            success = success and res.success
            results.append(res)
            if len(render_error) > 0:
              results.append(self.ErrorResultTuple("CASE_STDERR", 'Error when validating image for ' + image_path + ':' + "\n".join(render_error), ''))
          elif line == 'performance_start':
            # Performance format: One line that says performance, one line with all framerate samples, separated by space
            line = self.getNextLine(f)
            output = []
            while line != 'performance_end':
              output.append(line + '\n')
              line = self.getNextLine(f)
            res = self.getPerformanceResult(string.join(output))
            #success = success and res.success
            results.append(res)
          elif line == 'console':
            # Console format: one line that says console and nothing else. We need to get stderr and look through it for a line that says "console_start_<stepname>" and read it until we get to "console_end_<stepname>"
            
            output = []
            found = False
            ostream = self.std_err if testcase.console_ostream=='cerr' else self.std_out

            for err_line in ostream.splitlines(False):
              if not found:
                if err_line == "console_start_" + step_name:
                  found = True
              elif err_line == "console_end_" + step_name:
                break
              else:
                output.append(err_line + '\n')
            console_f = open(text_output_folder +'\\' + testcase.name + '_' + step_name + '_console.txt', 'w')
            console_f.writelines(output)
            console_f.flush()
            console_f.close()            
            res = self.getConsoleResult(baseline_folder + '\\' + testcase.name + '_' + step_name + "_console.txt", output)
            success = success and res.success
            results.append(res)
          elif line == 'custom_start':
            line = self.getNextLine(f, IgnoreNewline = False)
            output = []
            while line != 'custom_end':
              output.append(line + '\n')
              line = self.getNextLine(f, IgnoreNewline = False)
            res = self.getCustomResult(baseline_folder + '\\' + testcase.name + '_' + step_name+"_custom.txt", output)
            success = success and res.success
            results.append(res)
          else: # If we get anything else then we've reached the end of the current step and the start of the next one!
            step = TestResults.StepResultTuple(step_name, success, results)
            self.step_list.append(step)
            self.success = self.success and success
            break # goes out to the while step_name != True loop,
          line = self.getNextLine(f)
        step_name = line
      
      f.close()
    
    # Now check which steps, if any, were not run
    for step in testcase.expected_steps:
      step_was_run = False
      for result_tuple in self.step_list:
        if result_tuple[0] == step:
          step_was_run = True
          break
      if not step_was_run:
        self.step_list.append(self.StepResultTuple(step, False, [self.ErrorResultTuple("STEP_NOT_RUN", self.std_out + "\nThe step was not run.", "")]))
    # And finally save potential error step tuples, we put them at the start of the list.
    if (self.errors > 0):
      self.step_list.insert(0, self.StepResultTuple("", False, [self.ErrorResultTuple("CASE_STDERR", self.std_out + "\nTest returned errors", self.std_err)]))
      self.success = False
    elif not self.terminates_ok:
      self.step_list.insert(0, self.StepResultTuple("", False, [self.ErrorResultTuple("CASE_NOT_FINISHED", self.std_out + "\nTest didn't finish successfully (crashed, froze, or was otherwise interrupted.)", self.std_err)]))
      self.success = False
    