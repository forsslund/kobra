"""
This script is used to run tests on H3D scenes

Requirements:
  You must install Imagemagick (and add to PATH) in order for image comparison to function, otherwise rendering tests will always fail.
"""

import os, sys
import time
import platform
from threading  import Thread
import tempfile
import string
import argparse
import math
import signal
import glob
#import Image
if sys.version_info[0] >= 3:
  import configparser as ConfigParser
else:
  import ConfigParser
import json
import datetime
from collections import namedtuple
from inspect import getmembers, isfunction
import operator
from importlib import import_module
import ast
import re
import traceback

from TestResults import TestResults
from Variations import Option, Variation
from ProcessWrapper import *
import subprocess

parser = argparse.ArgumentParser(
  description='Runs python tests')
parser.add_argument('--workingdir', dest='workingdir', 
                    default=os.getcwd().replace('', ''),
                    help='The directory containing the test definitions.')
parser.add_argument('--processworkingdir', dest='processworkingdir', 
                    default=None,
                    help='The working directory for the test process. If omitted, will be set to base of X3D file to test')
parser.add_argument('--processpath', dest='processpath',
                    help='The path to the application used to run the tests. This is so you don''t need H3DLoad in your path',
                    default='')
parser.add_argument('--processname', dest='processname', 
                    default='H3DLoad',
                    help='The application used to run the tests.')
parser.add_argument('--processargs', dest='processargs',
                    default='',
                    help='Can be used to pass arguments to the process that you run. These will be put before the name of the .x3d file for the test that is being launched. This can be used to do things like running a python script instead of H3D by setting the processname to python.exe and the args to the path of the script you want to run.')
parser.add_argument('--output', dest='output',
                   default='output',
                   help='The directory to output the results to.')
parser.add_argument('--only_validate', dest='only_validate', action='store_true',
                    default=False,
                    help='Does not run the test cases, only goes through the output directory and compares the generated output. Will not report start/termination results. Can not be combined with only_generate.')
parser.add_argument('--only_generate', dest='only_generate', action='store_true',
                    default=False,
                    help='does not run validation on the generated output. Will not report rendering or performance comparison results. Can not be combined with only_validate.')
parser.add_argument('--timestamp', dest='timestamp', help='Format is YYYY-MM-DD HH:MM:SS. The time part of the datetime is optional. This datetime will be stored in the database for all the tests this script will run. If nothing is specified it will default to the current time.',
                    default=datetime.datetime.strftime(datetime.datetime.today(), "%Y-%m-%d %H:%M:%S"))                    
parser.add_argument('--dbhost', dest='dbhost', help='Address to database.')
parser.add_argument('--dbname', dest='dbname', help='Database name.')
parser.add_argument('--dbuser', dest='dbuser', help='Database user. Needs write-access to the database.')
parser.add_argument('--dbpass', dest='dbpass', help='Database password.')
parser.add_argument('--buildname', dest='buildname', help='The name of this build. All results from tests run by this script will be associated with this name together with the value of --hardwarename flag.',
                    default='Unknown')
parser.add_argument('--hardwarename', dest='hardwarename', help='The hardware the test was run on. All rsults will be associated with this name together with the value of --buildname. If not specified, the value will be "Unspecified".',
                    default='Unspecified')
parser.add_argument('--RunTestsDir', dest='RunTestsDir', help='The location of UnitTestsUtil.py and TestBoilerplate.py. This is for the cases where RunTests.py is being run from a different directory, for example for targeting a specific build of H3D.',
                    default=os.path.dirname(os.path.realpath(__file__)).replace('\\', '/'))
parser.add_argument('--case', dest='case', help='The name of one or more cases located in a TestDef somewhere in or below the workingdir. If this is specified then only these cases, out of all the cases in all the TestDefs, will be run', default='')

parser.add_argument('--testdefs', dest='testdefs', help='The name of one or more testdefs located somewhere in or below the workingdir. If this is specified then only cases in these TestDefs will be run. This can be combined with --case if you have multiple testdefs containing cases with the same name and only want to run one of them.', default='')
parser.add_argument('--resolution', dest='resolution', help='The resolution h3dload should be run at (only used for h3dload), in the format widthxheight, for example 800x600', default='640x480')
parser.add_argument('--inject_at_end_of_scene', dest='inject_at_end_of_scene', help='Specifies if the testing boilerplate nodes should be injected before </Scene> instead of the standard behaviour of injecting after <Scene>. For compatibility with projects that do search-and-replace inside the x3d file and might match information in one of the testing nodes if they come before nodes that are expected to be in Scene.', default=False)
parser.add_argument('--skipResultUpload', dest='skipResultUpload', action='store_true',help='Specifies if skip the uploading of result, this can be handy while doing simple test', default=False)
parser.add_argument('--skipSvnInfoOutput', dest='skipSvnInfoOutput', action='store_true',help='Specifies if skip extracting svn info of x3d and script file used in for the test', default=False)
parser.add_argument('--retentionTime', type=int, dest='retentionTime', help="How old test results are allowed to be (counted in days) before they are deleted automatically. If unspecified then no deletions will be made. If specified, will delete all test results and test runs that are older than [retentionTime] days before running all the tests. If it is not greater than 1 then no deletion will be done.", default=-1)
parser.add_argument('--description', dest='description', help="A description text that will be displayed on the results page. Can be used to describe useful information about this specific test run.", default=None)
args = parser.parse_known_args()[0]


def createFilename(orig_url, var_name):
  """ Returns a valid file name based on the provided path and variable name. """
  valid_chars = "-_.() %s%s" % (string.ascii_letters, string.digits)
  filename= orig_url + var_name
  return ''.join(c for c in filename if c in valid_chars).strip ( '.' )

def isError ( result ):
  """ Returns true if one of the possible errors occurred. """
  return (result.errors > 0 or not result.starts_ok or not result.terminates_ok) and not result.skipped

def getExitCode ( results ):
  """ Returns an exit code based on the test results. """
  for result in results:
    for variation in result:
      v= variation[0]
      r= variation[1]
      if isError ( r ):
        return 1
  return 0
  
   
class TestCaseRunner ( object ):

  def __init__ ( self, filepath, startup_time= 10, shutdown_time= 5, resolution=None, testable_callback= None, fake_fullscreen_due_to_PIL_bug = True, error_reporter = None):
    self.filepath= filepath
    self.startup_time= startup_time
    self.shutdown_time= shutdown_time
    self.testable_callback= testable_callback
    self.early_shutdown_file = '%s/test_complete' % (args.RunTestsDir)
    self.startup_time_multiplier = 1
    self.db = None
    # Used to get intermediate failed results in order to be able to cancel test early and still get some results.
    self.error_reporter = error_reporter

  def getProcess( self ):
    platform_system = platform.system()
    if platform_system == 'Windows':
      return ProcessWin32()
    elif platform_system == 'Linux':
      return ProcessUnix()

  def _countWarnings ( self, test_results, warnings_ignore_list = None ):
  
    # Open the ignore list file if it exists
    ignore_list = None
    if warnings_ignore_list:
      try:
        with open( warnings_ignore_list, "r" ) as file:
          ignore_list = file.readlines()
      except IOError:
        print("Failed to open warnings ignore list file " + warnings_ignore_list)
  
    found_result = re.split(r'((?:^\S*console_start.+$)|(?:^\S*console_end.+$))', test_results.std_out + test_results.std_err, flags=re.MULTILINE)
    haystack = ""
    inside_console_step = None
    # This code will fail if the first console_*** is not console_start. In
    # that case the only thing that happens is that there is a bit too much information in the end result.
    for i in found_result:
      if i.find("console_start") != -1:
        inside_console_step = True
      if not inside_console_step:
        haystack += i
      if i.find("console_end") != -1:
        inside_console_step = False
    
    # Returns true if the given line does not contain anything in the ignore list file
    def ignoreListFilter( line ):
      for ignore_line in ignore_list:
        if ignore_line.rstrip() in line:
          return False
      return True
      
    # Filter out all lines containing strings given in the ignore list file before counting warnings
    if ignore_list:
      haystack_list = haystack.split( '\n' )
      haystack_list[:] = [ x for x in haystack_list if ignoreListFilter( x ) ]
      haystack = '\n'.join( haystack_list )    
    
    haystack= haystack.lower()
    return ( haystack.count( "warning" ), haystack.count( "error" ) )

  def launchTest ( self, url, cwd, resolution=None, console_ostream='cerr' ):

    if h3d_process_name.lower() == "h3dload":
      if resolution:
        self.load_flags = ["--no-fullscreen","--screen=" + resolution]
      else:
        self.load_flags = ["--no-fullscreen","--screen=" + args.resolution]        
    else:
      self.load_flags = []

    process = self.getProcess()
    proc_cmd = os.path.join(args.processpath, h3d_process_name)
    proc_url = url

    if ' ' in proc_cmd:
      proc_cmd = '"' + proc_cmd + '"'
    if ' ' in proc_url:
      proc_url = '"' + proc_url + '"'

    os.environ['H3D_CONSOLE_OSTREAM'] = console_ostream

    if self.processargs != []:
      process.launch ( [proc_cmd] + self.processargs + self.load_flags + [proc_url], cwd)
    else:
      process.launch ( [proc_cmd] + self.load_flags + [proc_url], cwd)
    return process

  def testStartUp ( self, url, cwd, variation, resolution=None):
    """ Returns true if the example can be started. """
    
    if h3d_process_name.lower() == "h3dload":
      if resolution:
        self.load_flags = ["--no-fullscreen","--screen=" + resolution]
      else:
        self.load_flags = ["--no-fullscreen","--screen=" + args.resolution]        
    else:
      self.load_flags = []    

    process = self.getProcess()

    if self.processargs != []:
      return process.testLaunch ( [os.path.join(args.processpath, h3d_process_name)] + self.processargs + self.load_flags + [url], cwd, self.startup_time, self.shutdown_time, 1 if variation and variation.global_insertion_string_failed else self.startup_time_multiplier, self.early_shutdown_file )
    else:
      return process.testLaunch ( [os.path.join(args.processpath, h3d_process_name)] + self.load_flags + [url], cwd, self.startup_time, self.shutdown_time, 1 if variation and variation.global_insertion_string_failed else self.startup_time_multiplier, self.early_shutdown_file )
  
  def getExePath(self,program):
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None
  
  def runTestCase (self, filename, test_case, url, orig_url= None, var_name= "", variation = None):
    if orig_url is None:
      orig_url= url
    
    test_results= TestResults( )
    test_results.filename= filename
    test_results.name= test_case.name
    test_results.url= orig_url

    self.processargs = [arg for arg in (args.processargs.split(' ') + test_case.processargs.split(' ')) if arg != ""]

    self.startup_time = test_case.starttime
    self.shutdown_time = test_case.runtime
    if args.processworkingdir is None:
      cwd= os.getcwd()
    else:
      cwd= args.processworkingdir
      print("working dir for process is: " + cwd)
    filename= os.path.abspath ( url )
    
    if os.path.isfile( self.early_shutdown_file ):
      os.remove( self.early_shutdown_file )
    if os.path.isfile("childProcessIdList.txt"):
      os.remove( "childProcessIdList.txt" )

    process= self.launchTest ( url, cwd, test_case.resolution, test_case.console_ostream)

    time_slept = 0.0
    while time_slept < (self.startup_time ):
      time.sleep(0.5)
      time_slept += 0.5
      if not process.isRunning():
        break
    if not process.isRunning ():
      print("Test finished successfully after " + str(time_slept) + "s")
      test_results.std_out= process.getStdOut()
      test_results.std_err= process.getStdErr()
      if test_case.ignore_warnings:
        test_results.warnings, test_results.errors = [0,0]
      else:
        test_results.warnings, test_results.errors= self._countWarnings ( test_results, test_case.warnings_ignore_list )
      test_results.terminates_ok = os.path.isfile(self.early_shutdown_file)
      return test_results
   
    # We continue only if the process is still running after the startup time and sleep until the runtime runs out or until the process stops running
    time_slept = 0.0
    while time_slept < self.shutdown_time and process.isRunning():
      time.sleep(0.5)
      time_slept += 0.5

    if not process.isRunning ():
      print("Test finished successfully after " + str(self.startup_time + time_slept) + "s")

      # display warning if test finishes with (less than, or more than) this time remaining
      show_warning_secs = (10, 30)
      time_remaining = self.shutdown_time - time_slept
      if time_remaining < show_warning_secs[0]:
        print(( 
          "WARNING: Test finished with less than %.1f seconds (%.1fs) before timeout. "
          "Consider increasing timeout from %.1f to %.1f!" % 
          (show_warning_secs[0], time_remaining, self.shutdown_time, time_slept+show_warning_secs[0])))

      if time_remaining > show_warning_secs[1]:
        print(( 
          "NOTE: Test finished with more than %.1f seconds (%.1fs) before timeout. "
          "Consider reducing timeout from %.1f to %.1f!" % 
          (show_warning_secs[1], time_remaining, self.shutdown_time, time_slept+show_warning_secs[1])))

      test_results.terminates_ok= True
    else:
      print("Shutdown timeout hit, test looks like it crashed or froze.")
      test_results.terminates_ok= False
    # Now that we're done running the tests, regardless of if it failed or not we should kill the process and all its child processes (killing the process also triggers killing WERFault which ensures even crashed child processes will be properly terminated.
    try:
      print("killing processes")
      process.kill ()
      time_slept = 0
      self.shutdown_timeout = 60
      while time_slept < self.shutdown_timeout and process.isRunning():
        time.sleep(0.5)
        time_slept += 0.5
    except:
      pass
    # after process kill, additionally check if other processes started are killed
    # It seems sometime, process.kill() can not locate all child processes. 
    # Note: The solution only works on windows, and require additional work to store
    # process ids started in the case execution
    if platform.system() == 'Windows':
      if os.path.isfile("childProcessIdList.txt"):
        f = open("childProcessIdList.txt",'r')
        process_ids = f.read().split()
        f.close()
        for process_id in process_ids:
          process.killProcessThroughWMI(int(process_id))
    test_results.std_out= process.getStdOut()
    test_results.std_err= process.getStdErr()
    if test_case.ignore_warnings:
      test_results.warnings, test_results.errors = [0,0]
    else:
      test_results.warnings, test_results.errors= self._countWarnings ( test_results, test_case.warnings_ignore_list )
    return test_results


  def parseTestDefinitionFile(self, file_path):
    """
    Parses the specified test definition file and returns a list of namedtuples containing the following:      
      name: the name of this test case
      x3d: x3d file path
      baseline: baseline folder
      script: the test script
      All of these values default to None
    The list will contain one namedtuple for each Section in the specified definition file
    """
    confParser = ConfigParser.RawConfigParser(defaults={'x3d':None, 'baseline':None, 'script':None, 'runtime':1, 'starttime':1, 'fuzz': 2, 'threshold': 20, 'resolution': None, 'processargs': '', 'physics_engine': None, 'ignore_warnings': None, 'console_ostream': 'cerr'}, allow_no_value=True)
    try:
      confParser.read(file_path)
    except:
      print(sys.exc_info()[0])
      return None
    result = []
    description = ""
    for sect in confParser.sections():
      if sect == 'FileDescription':
        try:
          description = confParser.get(sect, "description")
        except:
          description = ''
      else:
        test_case = namedtuple('TestDefinition', ['name', 'filename', 'x3d', 'baseline', 'script', 'runtime', 'starttime', 'resolution', 'processargs', 'maxtrials', 'physics_engine', 'ignore_warnings', 'expected_steps', 'warnings_ignore_list', 'console_ostream'])
        test_case.name = sect
        test_case.x3d = confParser.get(sect, 'x3d')
        test_case.baseline = confParser.get(sect, 'baseline folder')
        test_case.script = confParser.get(sect, 'script')
        
        if test_case.script != "":
          test_case.expected_steps = []
          try:
            # Load the test script
            testdef_dir = os.path.dirname(file_path)
            script_path = os.path.join(testdef_dir,  test_case.script)
            if os.path.exists(script_path):
              script_file = open(script_path, "rb")
              tree = ast.parse(script_file.read())
              script_file.close()
              for item in tree.body:
                if isinstance(item, ast.FunctionDef) and 'decorator_list' in item._fields:
                  for decorator in item.decorator_list:
                    try:
                      if decorator.func.id in ['custom', 'console', 'screenshot', 'image', 'performance']:
                        test_case.expected_steps.append(item.name)
                    except:
                      pass
          except Exception as e:
            pass
        
        try: 
          test_case.runtime = confParser.getfloat(sect, 'timeout')
        except:
          test_case.runtime = 1
        try:
          test_case.starttime = confParser.getfloat(sect, 'starttime')
        except:
          test_case.starttime = 1
        try:
          test_case.fuzz = confParser.getfloat(sect, 'fuzz')
        except:
          test_case.fuzz = 2
        try:
          test_case.threshold = confParser.getfloat(sect, 'threshold')
        except:
          test_case.threshold = 20
        try:
          test_case.resolution = confParser.get(sect, 'resolution')
        except:
          test_case.resolution = None

        try:
          test_case.processargs = confParser.get(sect, 'arguments')
        except:
          test_case.processargs = ''
          
        try:
          test_case.maxtrials = confParser.getint(sect, 'maxtrials')
        except:
          test_case.maxtrials = 1

        try:
          test_case.physics_engine = confParser.get(sect, 'physics_engine')
        except:
          test_case.physics_engine = None

        try:
          test_case.ignore_warnings = confParser.getboolean(sect, 'ignore_warnings')
        except:
          test_case.ignore_warnings = False
          
        try:
          warnings_ignore_list = confParser.get(sect, 'warnings_ignore_list')
          test_case.warnings_ignore_list = os.path.join( os.path.dirname( file_path ), warnings_ignore_list )
        except:
          test_case.warnings_ignore_list = None

        try:
          test_case.console_ostream = confParser.get(sect, 'console_ostream')
        except:
          test_case.console_ostream = 'cerr'

        if args.case == "" or test_case.name in args.case.split():
          result.append(test_case)
    
    return result, description

  def processTestDef(self, file, testCase, results, directory):
    """
        
    """
    all_tests_successful = True
    
    output_dir = os.path.abspath(os.path.join(directory, "output"))
    rendering_dir = os.path.join(directory, output_dir, 'renderings')
    text_dir = os.path.join(directory, output_dir, 'text')
    for dir in [output_dir, rendering_dir, text_dir]:
      if not os.path.exists(dir):
        os.mkdir(dir)


    if os.path.exists(os.path.join(rendering_dir)):
      for file in os.listdir(rendering_dir):
        if file.startswith("diff_"):
          os.remove(os.path.join(rendering_dir, file))
                
    script = """
    <MetadataString DEF='TestCaseName' value='%s'/>
    <MetadataString DEF='TestCaseScriptFolder' value='%s'/>
    <MetadataString DEF='TestCaseScriptFilename' value='%s'/>
    <MetadataString DEF='TestCaseDefFolder' value='%s'/>
    <MetadataString DEF='TestBaseFolder' value='%s'/>
    <MetadataFloat DEF='StartTime' value='%f'/>
    <PythonScript DEF='TestScript' moduleName="TestBoilerplate" url='%s'></PythonScript>""" % (testCase.name,
                                                                  os.path.split(os.path.abspath(os.path.join(directory, testCase.script)))[0].replace('\\', '/'),
                                                                  os.path.splitext(os.path.split(testCase.script)[1])[0],
                                                                  os.path.abspath(directory).replace('\\', '/'),
                                                                  args.RunTestsDir,
                                                                  testCase.starttime,
                                                                  os.path.join(args.RunTestsDir, 'TestBoilerplate.py'))
    v = Variation (testCase.name, script)

    # use svn info to attempt to find the repo url of this test and its test script
    if args.skipSvnInfoOutput:
      svn_info_out = ""
      svn_url_x3d = ""
      svn_url_script = ""
    else:
      p = subprocess.Popen( 'svn info "' + os.path.join(directory, testCase.x3d) + '"', stdout=subprocess.PIPE, shell=False )
      svn_info_out, _ = p.communicate()
      svn_url_x3d = ""
      svn_url_script = ""
      if p.returncode != 0:
        print("Unable to obtain svn info for test x3d file, won't include it in results")
      else:
        #include it here
        for line in svn_info_out.split(b'\r\n'):
          if line.startswith(b"URL: "):
            svn_url_x3d = line[5:]
            break
      p = subprocess.Popen('svn info "' + os.path.join(directory, testCase.script) + '"', stdout=subprocess.PIPE, shell=False )
      svn_info_out, _ = p.communicate()
      if p.returncode != 0:
        print("Unable to obtain svn info for test x3d file, won't include it in results")
      else:
        #include it here
        for line in svn_info_out.split(b'\r\n'):
          if line.startswith(b"URL: "):
            svn_url_script = line[5:]
            break

    # Check if we should change the physics engine being used for this test and if so then use Variation's Option feature to ensure the test is run with that engine
    if not testCase.physics_engine is None:
      v.options.append ( Option ( ["RigidBodyCollection"], "physicsEngine", testCase.physics_engine ) )
      v.options.append ( Option ( ["PhysicsBodyCollection"], "physicsEngine", testCase.physics_engine ) )

    # Create a temporary x3d file containing our injected script

    original_path = os.path.join(directory, testCase.x3d)
    try:
      if os.path.exists(original_path+'_original.x3d'):
        print(original_path+'_original.x3d already exists! This probably means that RunTests was interrupted during a previous execution.')
        print("Restoring " + original_path + " before continuing...")
        try:
          if os.path.exists(original_path):
            os.remove(original_path)
          os.rename(original_path+'_original.x3d', original_path)
          print("Restored! Continuing the testing...")
        except:
          print("Failed to restore! Please check the files manually.")
          return
      if (os.path.exists(os.path.join(output_dir, "validation.txt"))):
        os.remove(os.path.join(output_dir, "validation.txt"))

      success, variation_path= self._createVariationFile ( v, os.path.join(directory, testCase.x3d))
      os.rename(original_path, original_path+'_original.x3d') # rename the original .x3d file
      os.rename(variation_path, original_path) # swap in our variation file for the original
    except:
      pass
    try:
      # Run the test
      for trial in range(testCase.maxtrials):
        print("NOTE: Running trial %d/%d..." % (trial+1, testCase.maxtrials))
        result = self.runTestCase (file, testCase, os.path.abspath(original_path), os.path.join(directory, testCase.x3d), v.name, v)
        if result.terminates_ok:
          print(result.std_err)
          print(result.std_out)
          break
        else:
          print("WARNING: Trial %d/%d failed!" % (trial+1, testCase.maxtrials))

      result.parseValidationFile(
        testCase, os.path.abspath( os.path.join( output_dir, "validation.txt" ) ),
        os.path.abspath( os.path.join( directory, testCase.baseline ) ),
        os.path.abspath( os.path.join( output_dir, "text" ) ), testCase.fuzz, testCase.threshold )
        
      if result.success and all_tests_successful:
        exitcode = 0
      else:
        exitcode = 1
    except Exception as e:
      print(str(e))
  

    try:
      os.remove(original_path)
    except:
      pass
    try:
      os.remove(variation_path)
    except:
      pass
    try:
      os.rename(original_path+'_original.x3d', original_path)
    except:
      pass

    if result:
      if sys.version_info[0] < 3 or isinstance( svn_url_x3d, str ):
        result.svn_url_x3d = svn_url_x3d
      else:
        result.svn_url_x3d = svn_url_x3d.decode("utf-8")

      if sys.version_info[0] < 3 or isinstance( svn_url_script, str ):
        result.svn_url_script = svn_url_script
      else:
        result.svn_url_script = svn_url_script.decode("utf-8")

    return result

  def processAllTestDefinitions ( self, directory= ".", output_dir= ".", fileExtensions= [".testdef"] ):
    """
        
    """
    try:
      os.makedirs(self.filepath)
    except:
      pass
     
    results = []

    if not args.skipResultUpload:
      if sys.version_info[0] >= 3:
        global MySQLdb
        import pymysql as MySQLdb
      else:
        global MySQLdb
        import MySQLdb
      self.build_name = args.buildname
      self.hardware_name = args.hardwarename
      self.ConnectDB()
      
      curs = self.db.cursor()
      curs.execute("SELECT id FROM servers WHERE build_name='%s' AND hardware_name='%s'" % (self.build_name, self.hardware_name))
      res = curs.fetchone()
      if res == None:
        print("Failed to obtain server id from db")
        return
      self.server_id = res[0]
    
    if args.retentionTime > 1 and not args.skipResultUpload:
      self.DeleteOldResults(args.retentionTime)
    
    all_tests_successful = True
    all_tests_run = True
    
    found_tests = False
    for root, dirs, files in os.walk(directory):
      for file in files:
        try:
          base, ext= os.path.splitext(file)
          if ext.lower() in fileExtensions:
            if args.testdefs == "" or base in args.testdefs.split():
              file_path= os.path.join(root,file)
              testCases, filedescription = self.parseTestDefinitionFile(file_path)
              for testCase in testCases:
                try:
                  if testCase != None and testCase.x3d != None and testCase.script != None:
                    found_tests = True
                    print("Testing: " + testCase.name)
                    case_results = self.processTestDef(file, testCase, results, root)
                    results.append(case_results)
                    if case_results != None:
                      all_tests_successful = all_tests_successful and case_results.success
                      all_tests_run = all_tests_run and case_results.terminates_ok
                    else:
                      all_tests_succesful = False
                      all_tests_run = False
                    testCase.filename = (os.path.relpath(file_path, directory)).replace('\'', '/') # This is used to set up the tree structure for the results page. It will store this parameter in the database as a unique identifier of this specific file of tests.
                    if not args.skipResultUpload and case_results != None:  
                      self.UploadResultsToSQL(testCase, case_results, root, filedescription)
                except Exception as e1:
                  print( "Something went wrong while handling test case " + str(testCase.name) + " in TestDef file: " + str(file) + " with exception message: " + str(e1) )
                  traceback.print_exc(file=sys.stderr)
        except Exception as e:
          print("processing defs in " + str(file)+" failed with exception: " + str(e) )
    if not found_tests:
      print("No valid tests found in: " + os.path.abspath(directory))
    return results, all_tests_successful, all_tests_run
  

  def _isTestable ( self, file_path ):
    
    if self.testable_callback:
      return self.testable_callback ( file_path )
    else:
      return True
  def _createVariationFile ( self, variation, file_path ):

    if sys.version_info[0] >= 3:
      orig_file= open ( file_path, 'r', encoding='utf-8', errors='ignore' )
      variation_file= tempfile.NamedTemporaryFile(mode='wb', dir=os.path.dirname(file_path), suffix='.x3d', delete= False )
      original_contents= orig_file.read()
      variation_contents= variation.parse ( original_contents, inject_at_end_of_scene=args.inject_at_end_of_scene)
      variation_file.write ( variation_contents.encode( 'utf-8', errors='ignore') )
      variation_file.close()
    else:
      orig_file= open ( file_path, 'r' )
      variation_file= tempfile.NamedTemporaryFile(dir=os.path.dirname(file_path), suffix='.x3d', delete= False )
      original_contents= orig_file.read()
      variation_contents= variation.parse ( original_contents, inject_at_end_of_scene=args.inject_at_end_of_scene)
      variation_file.write ( variation_contents )
      variation_file.close()

    return (variation_contents!=original_contents, variation_file.name)
 


  def ConnectDB(self):
    if self.db == None:
      try:
        print("Attempting to connect to results database at " + args.dbhost)
        self.db = MySQLdb.connect(host=args.dbhost, db=args.dbname, user=args.dbuser, passwd=args.dbpass)
        print("Connected!")
        self.db.autocommit = True
        curs = self.db.cursor()
        curs.execute("SELECT * FROM servers WHERE build_name='%s' AND hardware_name='%s'" % (self.build_name, self.hardware_name))
        if curs.fetchone() == None:
          curs.execute("INSERT INTO servers (build_name, hardware_name) VALUES ('%s', '%s')" % (self.build_name, self.hardware_name))
        curs.close()
        self.db.commit()
      except Exception as e:
        print((str(e)))        
        
        
  def DeleteOldResults(self, retentionTime):
    if retentionTime <= 1:
      print("Retention Time too low, needs to be greater than 1.")
      return
      
    self.ConnectDB()
    curs = self.db.cursor()
    
    curs.execute("SELECT id FROM test_runs WHERE timestamp < (CURRENT_DATE - interval " + str(retentionTime) + " day) AND allow_deletion = 'Y'")
    res = curs.fetchall()
    if len(res) > 0:
      print("found " + str(len(res)) + " test runs that were older than " + str(retentionTime) + " days, deleting them now...")
      for row in res:
        curs.execute("DELETE FROM error_results WHERE test_run_id = %d;" % row[0])
        curs.execute("DELETE FROM custom_results WHERE test_run_id = %d;" % row[0])
        curs.execute("DELETE FROM rendering_results WHERE test_run_id = %d;" % row[0])
        curs.execute("DELETE FROM performance_result_data WHERE performance_result_id in (SELECT id FROM performance_results WHERE test_run_id = %d);" % row[0])
        curs.execute("DELETE FROM performance_results WHERE test_run_id = %d;" % row[0])
        curs.execute("DELETE FROM test_runs WHERE id = %d" % row[0])
    
  def _truncateTextForSQL ( self, text ):
    """ Returns a truncate string for use as a TEXT field. 
    """

    # max length of a text field
    mysql_max_text_len = 65535

    # truncation warning message
    truncation_warning = "\n\nWARNING: log truncated here!"

    if len(text) > mysql_max_text_len:
      return text[:mysql_max_text_len-len(truncation_warning)] + truncation_warning
    else:
      return text
    
  def UploadResultsToSQL(self, testCase, case_results, output_dir, testfile_description):
    print("Uploading results for " + testCase.filename)
    self.ConnectDB()
    curs = self.db.cursor()
    
    # fetch the optional test_description.txt file from the directory
    testcat_description = ""
    if os.path.exists(os.path.join(args.workingdir, os.path.split(testCase.filename)[0], "description.txt")):
      description_file = open(os.path.join(args.workingdir, os.path.split(testCase.filename)[0], "description.txt"), "r")
      testcat_description = description_file.read()
    # check if there is a description for this directory already, if so replace it if it differs from this one
    curs.execute("SELECT id, description FROM test_categories WHERE path='%s'" % self.db.escape_string(os.path.split(testCase.filename)[0]))
    res = curs.fetchone()
    if res == None:
      #Doesn't exist, so upload it
      curs.execute("INSERT INTO test_categories (path, description) VALUES ('%s', '%s')" % (self.db.escape_string(os.path.split(testCase.filename)[0]), self.db.escape_string(testcat_description)))
    else:
      if res[1] != testcat_description:
        curs.execute("UPDATE test_categories SET description='%s' WHERE id=%d" % (self.db.escape_string(testcat_description), res[0]))

    # Then ensure the test file and test_case exists in the database
    curs.execute("SELECT id, description FROM test_files WHERE filename='%s'" % os.path.splitext(testCase.filename.replace("\\", "/"))[0])
    res = curs.fetchone()
    if res == None: # test_file doesn't exist, so add it
      curs.execute("INSERT INTO test_files (filename, description) VALUES ('%s', '%s')" % (os.path.splitext(testCase.filename.replace("\\", "/"))[0], self.db.escape_string(testfile_description)))
      curs.execute("SELECT id FROM test_files WHERE filename='%s'" % os.path.splitext(testCase.filename.replace("\\", "/"))[0])
      res = curs.fetchone()
    else:
      if res[1] != testfile_description:
        curs.execute("UPDATE test_files SET description='%s' WHERE id=%d" % (self.db.escape_string(testfile_description), res[0]))
    testfile_id = res[0]
    
    
    # Also add this test run to the database
    if not hasattr(self, "test_run_id"):
      curs.execute("INSERT INTO test_runs (timestamp, server_id, description) VALUES (\"" + args.timestamp + "\", %d, '%s')" % (self.server_id, args.description))
      self.test_run_id = curs.lastrowid
      if res == None:
        print("Failed to insert test run in db")
        return


    # Now ensure the test_case exists in the database
    curs.execute("SELECT test_cases.id, test_cases.svn_url_x3d, test_cases.svn_url_script FROM test_cases JOIN test_files WHERE case_name='%s'" % testCase.name)
    res = curs.fetchone()
    if res == None:
      curs.execute("INSERT INTO test_cases (case_name, svn_url_x3d, svn_url_script) VALUES ('%s', '%s', '%s')" % (testCase.name, case_results.svn_url_x3d, case_results.svn_url_script))
      curs.execute("SELECT id FROM test_cases WHERE case_name='%s'" % testCase.name)
      res = curs.fetchone()
    else:
      if (res[1] != case_results.svn_url_x3d) or (res[2] != case_results.svn_url_script):
        curs.execute("UPDATE test_cases SET svn_url_x3d='%s', svn_url_script='%s' WHERE test_cases.id=%d" % (case_results.svn_url_x3d, case_results.svn_url_script, res[0]))
    testcase_id = res[0]


    #StepResultTuple = namedtuple('StepResult', ['step_name', 'success', 'results']) # results is an array of results
    #PerformanceResultTuple = namedtuple("PerformanceResult", ['fps_min', 'fps_max', 'fps_avg', 'fps_mean', 'fps_full'])
    #RenderingResultTuple = namedtuple("RenderingResult", ['success', 'baseline_path','output_path', 'diff_path', 'diff_pixels', 'threshold'])
    #ConsoleResultTuple = namedtuple("ConsoleResult", ['success', 'baseline_path', 'output', 'diff'])
    #CustomResultTuple = namedtuple("CustomResult", ['success', 'baseline_path', 'output', 'diff'])
    for step in case_results.step_list:
      #First ensure the test step exists in the database
      curs.execute("SELECT test_steps.id FROM test_steps WHERE step_name='%s' AND test_case_id=%d" % (self.db.escape_string(step.step_name), testcase_id))
      res = curs.fetchone()
      if res == None:
        curs.execute("INSERT INTO test_steps (step_name, test_case_id) VALUES ('%s', %d)" % (self.db.escape_string(step.step_name), testcase_id))
        curs.execute("SELECT id FROM test_steps WHERE step_name='%s'" % self.db.escape_string(step.step_name))
        res = curs.fetchone()
      teststep_id = res[0]

      for result in step.results:
        if type(result).__name__ == 'ErrorResult':
          new_failure = 'Y'
          # Get the latest test run for this server to see if that error was gotten then. By not checking for the latest test run that contained the current test case we get a sort of quirk where if a test case previously had errors and then didn't get run and then gets run again it will display the error as new.
          curs.execute("SELECT id FROM test_runs WHERE id<%d and server_id=%d ORDER BY id DESC LIMIT 1;" % (self.test_run_id, self.server_id))
          res = curs.fetchone()
          if res != None:
            prev_testrun = res[0]
            curs.execute("SELECT id FROM error_results WHERE test_run_id=%d and file_id=%d and case_id=%d and step_id=%d ORDER BY test_run_id DESC LIMIT 1;" % (prev_testrun, testfile_id, testcase_id, teststep_id))
            res = curs.fetchone()
            if res == None:
              new_failure = 'Y'
            else:
              new_failure = 'N'
          # If no previous run for this server then set new_failure to Y
          else:
            new_failure = 'Y'
          curs.execute("INSERT INTO error_results (test_run_id, file_id, case_id, step_id, stdout, stderr, new_failure, error_type) VALUES (%d, %d, %d, %d, '%s', '%s', '%s', '%s')" % (self.test_run_id, testfile_id, testcase_id, teststep_id, self.db.escape_string(self._truncateTextForSQL(result.stdout)), self.db.escape_string(self._truncateTextForSQL(result.stderr)), new_failure, result.error_type))
        elif type(result).__name__ == 'ConsoleResult':
          output_string = ''
          for line in result.output:
            output_string += line
          if result.success:
            curs.execute("INSERT INTO console_results (test_run_id, file_id, case_id, step_id, success, output, new_failure, error_type) VALUES (%d, %d, %d, %d, 'Y', '%s', 'N', 'NO_ERROR')" % (self.test_run_id, testfile_id, testcase_id, teststep_id, self.db.escape_string(self._truncateTextForSQL(output_string))))
          else:
            # Try to fetch the previous test run to see if the failure is new
            curs.execute("SELECT success FROM console_results WHERE test_run_id<%d and file_id=%d and case_id=%d and step_id=%d ORDER BY test_run_id DESC LIMIT 1;" % (self.test_run_id, testfile_id, testcase_id, teststep_id))
            res = curs.fetchone()
            if res == None:
              new_failure = 'Y'
            else:
              if res[0] == 'Y' : # If the success value is true then the previous run succeeded so this failure is new
                new_failure = 'Y'
              else:
                new_failure = 'N'
            # Go read the baseline file so we can add that to the output
            if os.path.exists(result.baseline_path):
              f = open(result.baseline_path)
              baseline_string = f.read()
              f.close()
              curs.execute("INSERT INTO console_results (test_run_id, file_id, case_id, step_id, success, output, baseline, diff, new_failure, error_type) VALUES (%d, %d, %d, %d, 'N', '%s', '%s', '%s', '%s', '%s')" % (self.test_run_id, testfile_id, testcase_id, teststep_id, self.db.escape_string(self._truncateTextForSQL(output_string)), self.db.escape_string(baseline_string), self.db.escape_string(self._truncateTextForSQL(result.diff)), new_failure, result.error_type))
            else:
              curs.execute("INSERT INTO console_results (test_run_id, file_id, case_id, step_id, success, output, baseline, diff, new_failure, error_type) VALUES (%d, %d, %d, %d, 'N', '%s', NULL, NULL, '%s', '%s')" % (self.test_run_id, testfile_id, testcase_id, teststep_id, self.db.escape_string(self._truncateTextForSQL(output_string)), new_failure, result.error_type))
            
        elif type(result).__name__ == 'CustomResult':
          output_string = ''
          for line in result.output:
            output_string += line
          if result.success:
            curs.execute("INSERT INTO custom_results (test_run_id, file_id, case_id, step_id, success, output, new_failure, error_type) VALUES (%d, %d, %d, %d, 'Y', '%s', 'N', 'NO_ERROR')" % (self.test_run_id, testfile_id, testcase_id, teststep_id, self.db.escape_string(self._truncateTextForSQL(output_string))))
          else:
            # Try to fetch the previous test run to see if the failure is new
            curs.execute("SELECT success FROM custom_results WHERE test_run_id<%d and file_id=%d and case_id=%d and step_id=%d ORDER BY test_run_id DESC LIMIT 1;" % (self.test_run_id, testfile_id, testcase_id, teststep_id))
            res = curs.fetchone()
            if res == None:
              new_failure = 'Y'
            else:
              if res[0] == 'Y' : # If the success value  is true then the previous run succeeded so this failure is new
                new_failure = 'Y'
              else:
                new_failure = 'N'          
            # Go read the baseline file so we can add that to the output
            if os.path.exists(result.baseline_path):
              f = open(result.baseline_path)
              baseline_string = f.read()
              f.close()
            else:
              baseline_string = 'Baseline not found'
            curs.execute("INSERT INTO custom_results (test_run_id, file_id, case_id, step_id, success, output, baseline, diff, new_failure, error_type) VALUES (%d, %d, %d, %d, 'N', '%s', '%s', '%s', '%s', '%s')" % (self.test_run_id, testfile_id, testcase_id, teststep_id, self.db.escape_string(self._truncateTextForSQL(output_string)), self.db.escape_string(baseline_string), self.db.escape_string(self._truncateTextForSQL(result.diff)), new_failure, result.error_type))

        elif type(result).__name__ == 'PerformanceResult':
          # Insert the performance results, but only if all the tests in this step succeeded!
          if step.success:
#            curs.execute("INSERT INTO performance_results (test_run_id, file_id, case_id, step_id, min_fps, max_fps, avg_fps, mean_fps, full_case_data) VALUES (%d, %d, %d, %d, %s, %s, %s, %s, '%s')" % (self.test_run_id, testfile_id, testcase_id, teststep_id, result.fps_min, result.fps_max, result.fps_avg, result.fps_mean, result.fps_full))
            curs.execute("INSERT INTO performance_results (test_run_id, file_id, case_id, step_id, full_profiling_data) VALUES (%d, %d, %d, %d, '%s')" % (self.test_run_id, testfile_id, testcase_id, teststep_id, self._truncateTextForSQL(result.profiling_data_full)))
            perf_res_id = curs.lastrowid
            for data in result.profiling_data_lines:
              curs.execute("INSERT INTO performance_result_data (performance_result_id, level, identifier, mean, percent) VALUES (%d, %d, '%s', %f, %f)" % (perf_res_id, int(data.level), data.id, float(data.mean), float(data.percent)) )

        elif type(result).__name__ == 'RenderingResult':
            step_name = step.step_name
            
            # Fetch the current baseline so we can compare it to the one in the database
            if os.path.exists(result.baseline_path):
              baseline_file = open(result.baseline_path, 'rb')
              baseline_image = baseline_file.read()
              baseline_file.close()
              curs.execute("SELECT id, image FROM rendering_baselines WHERE file_id=%s AND case_id=%s AND step_id=%s ORDER BY timestamp DESC LIMIT 1" % (testfile_id, testcase_id, teststep_id))
              res = curs.fetchone()
              if res == None or res[1] != baseline_image:
                curs.execute(("INSERT INTO rendering_baselines (file_id, case_id, step_id, timestamp, image) VALUES (%d, %d, %d, '%s'" % (testfile_id, testcase_id, teststep_id, args.timestamp)) + ", _binary %s)", [baseline_image,])
#              elif res[1] != baseline_image:
#                curs.execute("INSERT into rendering_baselines  image=%s WHERE id=%s", [baseline_image, res[0]])
            else:
              print("Unable to find baseline at " + result.baseline_path + ", can't upload.")
              
            if not result.success: #validation failed, if possible we should upload both the rendering and the diff
              # Try to fetch the previous test run to see if the failure is new
              curs.execute("SELECT success FROM rendering_results WHERE test_run_id<%d and file_id=%d and case_id=%d and step_id=%d ORDER BY test_run_id DESC LIMIT 1;" % (self.test_run_id, testfile_id, testcase_id, teststep_id))
              res = curs.fetchone()
              if res == None:
                new_failure = 'Y'
              else:
                if res[0] == 'Y' : # If the success value  is true then the previous run succeeded so this failure is new
                  new_failure = 'Y'
                else:
                  new_failure = 'N'  
                            
              if result.diff_path != '':
                diff_file = open(result.diff_path, 'rb')
                diff_image = diff_file.read()
                diff_file.close()
              else:
                diff_image = 'NULL'
              if result.output_path != '':
                output_file = open(result.output_path, 'rb')
                output_image = output_file.read()
                output_file.close()
              else:
                output_image = 'NULL'
              curs.execute("INSERT INTO rendering_results (test_run_id, file_id, case_id, step_id, new_failure, error_type, diff_pixels, threshold, success, output_image, diff_image) VALUES (%d, %d, %d, %d, '%s', '%s', %d, %d" % (self.test_run_id, testfile_id, testcase_id, teststep_id, new_failure, result.error_type, result.diff_pixels, result.threshold) + ", 'N', _binary %s, _binary %s)", [output_image, diff_image]);
            else:
              curs.execute("INSERT INTO rendering_results (test_run_id, file_id, case_id, step_id, success, new_failure, error_type, diff_pixels, threshold) VALUES (%d, %d, %d, %d, 'Y', 'N', 'NO_ERROR', %d, %d)" % (self.test_run_id, testfile_id, testcase_id, teststep_id, result.diff_pixels, result.threshold));

    curs.close()
    self.db.commit()




   
class TestReport ( object ):
  
  def reportResults ( self, results ):
    output= ""
    if args.only_generate:
      return output
    all_renderings_succeeded = True

    for result in results:
      for variation in result:
        v= variation[0]
        r= variation[1]
        output+= "Test results for: " + str(r.name) + " (" + v.name + ")\n"
        output+= "\n"
        if not args.only_validate:
          output+= "Started OK    : %s\n" % str(r.starts_ok)
          output+= "Exited OK     : %s\n" % str(r.terminates_ok)
        if r.result_type == 'rendering':
          output+= "renderings:\n"
          for i in range(0, len(r.renderings)):
            output+= "  %s [%s]\n" % (r.step_names[i], ("OK" if r.renderings_ok[i] else "Failed"))
          if False in r.renderings_ok:
            all_renderings_succeeded = False

    if r.result_type == 'rendering':
      output+= "\n%s\n" % ("All rendering comparisons OK!" if all_renderings_succeeded else "One or more rendering comparisons Failed!")
    return output
    
class TestReportHTML ( object ):
  
  def __init__ ( self, filepath, only_failed= False ):
    self.filepath= filepath
    self.only_failed= only_failed
  
  def reportResults ( self, results ):
  
    try:
      os.makedirs(self.filepath)
    except:
      pass
    
    f= open ( os.path.join(self.filepath,'default.css'), 'w' )
    f.write ( self._generateCSS() )
    f.close()
    
    f= open ( os.path.join(self.filepath,'index_failed.html' if self.only_failed else 'index.html'), 'w' )
  
    output= """
<html>
  <head>
    <title>Test results</title>
    <link rel="stylesheet" href="default.css" type="text/css" />
  </head>
  <body>
    <h1>Test results</h1>
    %s
    <table>
""" % ("<h2>Failed only</h2>" if self.only_failed else "")
  
    skipped_list= []
  
    for result in results:
      if result[0][1].skipped:
        skipped_list.append ( result[0][1] )
    
      if self._includeResult ( result ):
      
        output+= "<tr>"
        r= result[0][1]
        output+= "<td><h3>" + r.name + "</h3>\n"
        
        for variation in result:
          v= variation[0]
          r= variation[1]
      
          report_path= os.path.join (
            self.filepath,
            v.name + "_report.html" )
      
          output+= "<td>"
          output+= "<h3>" + v.name + ( "" if r.created_variation else " (Unchanged file)" ) + "</h3>"
          if r.test_type == 'rendering':
            rendering = self._getFirstErrorrendering( r )
            if rendering == "":
              rendering = self._getrendering( r, count=1 )
          else:
            rendering = self._getrendering( r, count=1 )

          output+= rendering
          if not args.only_validate:
            output+= "<p>Start: " + self._getStatus ( r.started_ok ) + "; "
            output+= "Exit:" + self._getStatus ( r.terminated_ok ) + "; "
          if not args.only_generate:
            output+= "renderings: " + self._getrenderingStatus( r.renderings_ok ) + "</p>\n"
          output+= "<p>Errors: " + self._getErrorCount ( r.errors, 'fail' ) + "; "
          output+= "Warnings:" + self._getErrorCount ( r.warnings, 'warn' ) + "</p>\n"
          if not args.only_generate and r.has_fps:
            output+= "<p>Graphics frame rate: " + self._getFPS( r.fps_mean ) + "</p>\n"
          output+= "<p>[<a href='" + os.path.split(report_path)[1] + "'>details</a>]</p>"
          output+= "</td>"
          
          f1= open ( report_path, 'w' )
          f1.write ( self._reportTestcase ( r ) )
          f1.close()
          
        output+= "</tr>"
      
    output+= """
    </table>
    <h2>Skipped</h2>
    <ul>
    """
    if skipped_list:
      for r in skipped_list:
        output+= "<li>%s [%s]</li>" % (r.name, r.url)
    else:
      output+= "<li>None</li>"
      
    output+= """
    </ul>
  </body>
</html>"""
      
    f.write ( output )
    f.close()
      
    return output
    
  def _includeResult ( self, result ):
    for variation in result:
      if self._includeVariation ( variation[1] ):
        return True
    return False
    
  def _includeVariation ( self, variation ):
    return (variation.errors > 0 or variation.warnings > 0 or not variation.started_ok or not variation.terminated_ok or not self.only_failed) and not variation.skipped

  def _getThumbnail(self, rendering_path, thumb_dir):
#    if not os.path.exists(rendering_path):
#      return ""
#    else:
#      im = Image.open(rendering_path)
#      im.thumbnail((256,256), Image.ANTIALIAS)
#      rendering_filename = os.path.split(rendering_path)[1]
#      thumb_path = os.path.join(thumb_dir, os.path.splitext(rendering_filename)[0] + "_thumb.png")
#      im.save(thumb_path, "PNG")
#      return thumb_path
    pass
    
  def _reportTestcase ( self, result ):   

    output= """
<html>
  <head>
    <title>%s</title>
    <link rel="stylesheet" href="default.css" type="text/css" />
  </head>
  <body>
    <h1>%s</h1>
    <p>%s</p>
    %s
    <h2>Startup</h2>
    %s
    <h2>Shutdown</h2>
    %s
    <h2>Standard output</h2>
    <pre>%s</pre>
    <h2>Standard error</h2>
    <pre>%s</pre>""" % (
      result.name,
      result.name, result.url,
      self._getrendering ( result, count = (1 if result.test_type == 'performance' else 0)),
      self._getStatus ( result.started_ok ), 
      self._getStatus ( result.terminated_ok ),
      result.std_out,
      result.std_err)
    if not args.only_generate and result.has_fps:
      """<h2>Graphics frame rate</h2>
      <pre>Min: %s</pre>
      <pre>Max: %s</pre>
      <pre>Avg: %s</pre>
      <pre>Mean: %s</pre>""" % (self._getFPS( result.fps_min ),
      self._getFPS( result.fps_max ),
      self._getFPS( result.fps_avg ),
      self._getFPS( result.fps_mean ))

    if result.variation and result.variation.name != "Default":
      output = output + \
               """  <h2>Variation settings:</h2>"""
      for i, option in enumerate( result.variation.options ):
        if option.field_name != "":
          output = output + \
                   "    <h3>Option %s:</h2>\n" % str(i) + \
                   "    <pre>Node: " + self._getFPS( ", ".join( option.node_type ) ) + "</pre>\n" + \
                   "    <pre>Field name: " + self._getFPS( option.field_name ) + "</pre>\n" + \
                   "    <pre>Field value: " + self._getFPS( option.field_value ) + "</pre>\n"

    output = output + \
"""  </body>
</html>"""
    return output
    
  def _generateCSS ( self ):
    return """
body {
  font-family: sans-serif;
}

.success {
  color: green;
}

.warn {
  color: orange;
}

.fail {
  color: red;
  font-weight: bold;
}

td, th {
  width: 280px;
}

img {
  float: left;
}

"""
    
  def _getStatus ( self, status ):
    return "<span class='success'>OK</span>" if status else "<span class='fail'>FAIL</span>"
  
  def _getrenderingStatus ( self, status_list):
    return "<span class='success'>OK</span>" if not False in status_list else "<span class='fail'>FAIL</span>"
    
  def _getrendering ( self, result, count = 0 ):
    """
    Iterates through the renderings from a testcase result and returns HTML code for them.
    If count is not > 0 then it will only get that many renderings. Otherwise it will get all available renderings.
    """
    res = "<p>"
    for i in range(0, len(result.renderings)):
      if result.test_type == 'rendering' and result.rendering_diffs[i] != "":
        img_path = result.rendering_diffs[i]
      else:
        img_path = result.renderings[i]
      thumb_path = self._getThumbnail(img_path, self.filepath)
      res += "<a href='" + os.path.relpath(img_path, self.filepath) + "'><image src='" + os.path.relpath(thumb_path, self.filepath) + "' /></a>"
      count -= 1
      if count == 0:
        break
    res += "</p>\n"
    return res

  def _getFirstErrorrendering( self, result ):
    """ Iterates through all the test steps of a rendering test and returns HTML code for the first one that failed and has a diff image. """
    for i in range(0, len(result.renderings_ok)):
      if not result.renderings_ok[i] and result.rendering_diffs[i] != '':
        img_path = result.rendering_diffs[i]
        thumb_path = self._getThumbnail(img_path, self.filepath)
        return "<p><a href='" + os.path.relpath(img_path, self.filepath) + "'><image src='" + os.path.relpath(thumb_path, self.filepath) + "' /></a></p>\n"
    return ""
      
  def _getErrorCount ( self, nrErrors, errorType ):
    return ("<span class='success'>%s</span>" if nrErrors==0 else "<span class='"+errorType+"'>%s</span>") % nrErrors
  
  def _getFPS ( self, fps ):
    return ( "<span class='success'>%s</span>" % fps ) if fps != "" else "<span class='fail'>-</span>"
        

 

print("")
print("WARNING: Do not change the window focus, or perform other input until the test is complete!")
print("")


h3d_process_name = args.processname

exitCode = 0

def isTestable ( file_name , files_in_dir):
  return True

try:
  print("Running these tests using: " + subprocess.check_output('where.exe ' + ('"'+ args.processpath + '":' if (args.processpath != "") else "") + h3d_process_name).decode("utf-8") ) # Run our test script and wait for it to finish executing
except Exception as e:
  print(h3d_process_name + " not found, tests may not be run: " + str(e))



tester= TestCaseRunner( os.path.join(args.workingdir, ""), startup_time= 5, shutdown_time= 5, testable_callback= isTestable, error_reporter=None)
results, all_tests_successful, all_tests_run = tester.processAllTestDefinitions(directory=os.path.abspath(args.workingdir), output_dir=args.output)

if not all_tests_run:
  print("Error: Some tests were not run!")
  exitCode = 2
elif not all_tests_successful:
  print("Warning: One or more tests failed!")
  exitCode = 1


exit(exitCode)
