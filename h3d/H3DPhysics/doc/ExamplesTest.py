"""
This script finds runs and generates screenshots and reports for H3DPhysics 
examples.

An HTML document is generated that shows the results from each example file
in each physics engine.

Platform notes:
 Currently only implemented for Windows. 
 Screen capture and process management is Windows specific.

Additional requirements:
  * PIL: http://www.pythonware.com/products/pil/
 
usage: examples_test.py [-h] [--output OUTPUT] path
 
optional arguments:
  -h, --help       show this help message and exit
  --path PATH      root path to search for files
  --output OUTPUT  path to save output to
 
"""

import subprocess
from PIL import ImageGrab
from PIL import Image
import time
import os, sys
from Queue import Queue, Empty
from threading  import Thread
import tempfile
import string
import re
import argparse

import win32api
import win32com.client

parser = argparse.ArgumentParser(
  description='Finds runs and generates screenshots and reports for H3DPhysics examples.')
parser.add_argument('--path', dest='path',
                   default='../examples',
                   help='root path to search for files')
parser.add_argument('--output', dest='output',
                   default='examples_report',
                   help='path to save output to')
args = parser.parse_args()

def createFilename(orig_url, var_name):
  valid_chars = "-_.() %s%s" % (string.ascii_letters, string.digits)
  filename= orig_url + var_name
  return ''.join(c for c in filename if c in valid_chars).strip ( '.' )

def getExitCode ( results ):
  """ Returns an exit code based on the test results. """

  for result in results:
    for variation in result:
      v= variation[0]
      r= variation[1]
      if isError ( r ):
        return 1
  return 0
  
def isError ( result ):
  return (result.errors > 0 or result.warnings > 0 or not result.started_ok or not result.terminated_ok) and not result.skipped
  
class Process ( object ):
  
  def launch ( self, command, cwd ):
    pass
    
  def testLaunch ( self, command, cwd, startup_time, shutdown_time ):
    pass
    
  def isRunning ( self ):
    pass
    
  def sendKey ( self, key ):
    pass
    
  def kill ( self ):
    pass
    
  def getStdOut ( self ):
    pass
    
  def getStdErr ( self ):
    pass
    
class ProcessWin32 ( Process ):

  def launch ( self, command, cwd ):
  
    def enqueue_output ( stdout, stderr, std_out_q, std_err_q ):
      for line in iter(stdout.readline, b''):
        std_out_q.put(line)
      stdout.close()
      for line in iter(stderr.readline, b''):
        std_err_q.put(line)
      stderr.close()
  
    self.process_name= command[0]
  
    shell = win32com.client.Dispatch("WScript.Shell")

    import ctypes
    ctypes.windll.kernel32.SetErrorMode(0)

    self.process= subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd= cwd)
    
    self.std_out_q = Queue()
    self.std_err_q = Queue()
    self.monitor_output_thread = Thread(target= enqueue_output, args=(self.process.stdout, self.process.stderr, self.std_out_q, self.std_err_q))
    self.monitor_output_thread.start()
    
  def testLaunch ( self, command, cwd, startup_time, shutdown_time ):
  
    import ctypes
    SEM_NOGPFAULTERRORBOX = 0x0002 # From MSDN
    ctypes.windll.kernel32.SetErrorMode(SEM_NOGPFAULTERRORBOX)
    subprocess_flags = 0x8000000 #win32con.CREATE_NO_WINDOW?
    
    self.process_name= command[0]
    
    self.process= subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd= cwd, creationflags= subprocess_flags )
    time.sleep(startup_time)
    success= self.isRunning()
    if success:
      self.kill()
      time.sleep(shutdown_time)
    return success
    
  def isRunning ( self ):
    # for Windows XP use this
    query_process = "tasklist /NH /FI \"IMAGENAME EQ "+ self.process_name +"\""
    
    # for Windows Vista use this
    #query_process = 'query process'
    
    #for Linux use this
    #query_process = 'ps -Af | grep ' + self.process_name + 
    
    p = subprocess.Popen( query_process, stdout=subprocess.PIPE, shell=True )
    output = p.communicate()
    
    return output[0].find(self.process_name) >= 0
    
  def sendKey ( self, key ):
    shell = win32com.client.Dispatch("WScript.Shell")
    shell.AppActivate ( "H3D" )
    time.sleep ( 0.1 )
    shell.SendKeys(key)
    
  def kill ( self ):
    # for Windows XP
    kill_process = "taskkill /F /im "+ self.process_name
    os.system( kill_process )
    
    # If the test crashed, close the crash dialog box too
    shell = win32com.client.Dispatch("WScript.Shell")
    shell.AppActivate ( "H3DLoad.exe" )
    shell.SendKeys("{Esc}")
    
  def getStdOut ( self ):
    output= ""
    try:
      while not self.std_out_q.empty():
        output+= self.std_out_q.get(timeout=.1)
    except Empty:
      pass
    return output
    
  def getStdErr ( self ):
    output= ""
    try:
      while not self.std_err_q.empty():
        output+= self.std_err_q.get(timeout=.1)
    except Empty:
      pass
    return output

class Option ( object ):
  
  def __init__ ( self, node_type, field_name, field_value ):
    self.node_type= node_type
    self.field_name= field_name
    self.field_value= field_value
    
class Variation ( object ):

  def __init__ ( self, name ):
    self.name= name
    self.options= []
    
  def parse ( self, input ):
    replace_with= ""
    output= input
    
    def replace_callback(r):
      if r.group(1):
        return "<%s %s%s=\"%s\"" % (r.group(1),r.group(2),r.group(3),replace_with)
      else:
        return r.group(0)
  
    
    for o in self.options:
      for node_type in o.node_type:
        reg = re.compile(r'<('+node_type+r')\s([^>]*)('+o.field_name+r')\s*=\s*\'([^\']*)\'|"([^"]*)"', re.DOTALL)
        replace_with= o.field_value
        output = reg.sub(replace_callback, output)
      
    return output
    
class TestResults ( object ):
  
  def __init__ ( self ):
    self.name= ""
    self.url= ""
    self.std_out= ""
    self.std_err= ""
    self.started_ok= False
    self.terminated_ok= False
    self.screenshot= ""
    self.screenshot_thumb= ""
    self.warnings= 0
    self.errors= 0
    self.skipped= False
    self.created_variation= False
    
class TestExamples ( object ):

  def __init__ ( self, filepath, startup_time= 10, shutdown_time= 5 ):
    self.filepath= filepath
    self.startup_time= startup_time
    self.shutdown_time= shutdown_time

  def launchExample ( self, url, cwd ):
    process= ProcessWin32()
    process.launch ( ["H3DLoad.exe", "-f", url], cwd )
    return process
    
  def testStartUp ( self, url, cwd ):
    """ Returns true if the example can be started. """
    
    process= ProcessWin32()
    return process.testLaunch ( ["H3DLoad.exe", "-f", url], cwd, self.startup_time, self.shutdown_time )
    
  def testExample ( self, name, url, orig_url= None, var_name= "" ):
    
    if orig_url is None:
      orig_url= url
    
    test_results= TestResults()
    test_results.name= name
    test_results.url= orig_url
    
    cwd= os.path.split ( orig_url )[0]
    
    test_results.started_ok= self.testStartUp ( url, cwd )
    
    process= self.launchExample ( url, cwd )
    time.sleep(self.startup_time)
    if not process.isRunning ():
      test_results.std_out= process.getStdOut()
      test_results.std_err= process.getStdErr()
      test_results.warnings, test_results.errors= self._countWarnings ( test_results )
      return test_results
    
    if test_results.started_ok:
      im= ImageGrab.grab()
      test_results.screenshot= os.path.join(self.filepath,createFilename(orig_url, var_name)+".jpg")
      im.save(test_results.screenshot, "JPEG")
      
      im.thumbnail((256,256), Image.ANTIALIAS)
      test_results.screenshot_thumb= os.path.join(self.filepath,createFilename(orig_url, var_name)+"_thumb.jpg")
      im.save(test_results.screenshot_thumb, "JPEG")
    
      process.sendKey ( "{ESC}" )
      time.sleep(self.shutdown_time)
    
    if not process.isRunning ():
      test_results.terminated_ok= True
      test_results.std_out= process.getStdOut()
      test_results.std_err= process.getStdErr()
      test_results.warnings, test_results.errors= self._countWarnings ( test_results )
      return test_results
    else:
      process.kill ()
      time.sleep(self.shutdown_time)
      test_results.std_out= process.getStdOut()
      test_results.std_err= process.getStdErr()
      test_results.warnings, test_results.errors= self._countWarnings ( test_results )
      return test_results
  
  def _countWarnings ( self, test_results ):
    haystack= test_results.std_out + test_results.std_err
    haystack= haystack.lower()
    return ( haystack.count ( "warning" ), haystack.count ( "error" ) )
    
  
  def testAllExamples ( self, variations= None, directory= ".", fileExtensions= [".x3d"] ):
 
    try:
      os.makedirs(self.filepath)
    except:
      pass
 
    results= []
 
    for root, dirs, files in os.walk(directory):
      for file in files:
        base, ext= os.path.splitext(file)
        if ext.lower() in fileExtensions:
          file_path= os.path.join(root,file)
          if self._isTestable ( file_path ):
            print "Testing: " + file_path
            
            if variations:
              variation_results= []
              for v in variations:
                print "Variation: " + v.name
                success, variation_path= self._createVariation ( v, file_path )
                result= self.testExample ( file, variation_path, file_path, v.name )
                result.created_variation= success
                variation_results.append ( ( v, result ) )
                os.remove ( variation_path )
              results.append ( variation_results )
            else:
              results.append ( [( None, self.testExample ( file, file_path ) )] )
          else:
            # Test skipped
            result= TestResults()
            result.name= file
            result.url= file_path
            result.skipped= True
            if variations:
              variation_results= []
              for v in variations:
                variation_results.append ( (v, result) )
              results.append ( variation_results )
            else:
              results.append ( result )
          
    return results
  
  def _isTestable ( self, file_path ):
    
    f= open ( file_path, 'r' )
    contents= f.read ()
    f.close()
    return ( 
      contents.find ( 'RigidBodyCollection' ) > 0 or 
      contents.find ( 'PhysicsBodyCollection' ) > 0 )
  
  def _createVariation ( self, variation, file_path ):
    
    orig_file= open ( file_path, 'r' )
    
    variation_file= tempfile.NamedTemporaryFile ( delete= False )
    original_contents= orig_file.read()
    variation_contents= variation.parse ( original_contents )
    variation_file.write ( variation_contents )
    variation_file.close()
    
    return (variation_contents!=original_contents, variation_file.name)
  
class TestReport ( object ):
  
  def reportResults ( self, results ):
    output= ""
  
    for result in results:
      for variation in result:
        v= variation[0]
        r= variation[1]
        output+= "Test results for: " + str(r.name) + " (" + v.name + ")\n"
        output+= "\n"
        output+= "Started OK    : %s\n" % str(r.started_ok)
        output+= "Exited OK     : %s\n" % str(r.terminated_ok)
      
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
            createFilename(r.url, v.name) + "_report.html" )
      
          output+= "<td>"
          output+= "<h3>" + v.name + "</h3>"
          output+= self._getThumbnail ( r )
          output+= "<p>Start: " + self._getStatus ( r.started_ok ) + "; "
          output+= "Exit:" + self._getStatus ( r.terminated_ok ) + "</p>\n"
          output+= "<p>Errors: " + self._getErrorCount ( r.errors, 'fail' ) + "; "
          output+= "Warnings:" + self._getErrorCount ( r.warnings, 'warn' ) + "</p>\n"
          output+= "<p>[<a href='" + os.path.split(report_path)[1] + "'>details</a>]</p>"
          output+= "</td>"
          
          f1= open ( report_path, 'w' )
          f1.write ( self._reportExample ( r ) )
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
    
  def _reportExample ( self, result ):
  
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
    <pre>%s</pre>
  </body>
</html>""" % (
      result.name,
      result.name, result.url,
      self._getThumbnail ( result ),
      self._getStatus ( result.started_ok ), 
      self._getStatus ( result.terminated_ok ),
      result.std_out,
      result.std_err )
      
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
  width: 260px;
}

"""
    
  def _getStatus ( self, status ):
    return "<span class='success'>OK</span>" if status else "<span class='fail'>FAIL</span>"
    
  def _getThumbnail ( self, result ):
    if result.screenshot != "":
      return "<p><a href='" + os.path.split(result.screenshot)[1] + "'><image src='" + os.path.split(result.screenshot_thumb)[1] + "' /></a></p>\n"
    else:
      return ""
      
  def _getErrorCount ( self, nrErrors, errorType ):
    return ("<span class='success'>%s</span>" if nrErrors==0 else "<span class='"+errorType+"'>%s</span>") % nrErrors
      
ode_variation= Variation ( "ODE" )
ode_variation.options.append ( Option ( ["RigidBodyCollection","PhysicsBodyCollection"], "physicsEngine", "ODE" ) )

bullet_variation= Variation ( "Bullet" )
bullet_variation.options.append ( Option ( ["RigidBodyCollection","PhysicsBodyCollection"], "physicsEngine", "Bullet" ) )

physx_variation= Variation ( "PhysX" )
physx_variation.options.append ( Option ( ["RigidBodyCollection","PhysicsBodyCollection"], "physicsEngine", "PhysX" ) )

physx3_variation= Variation ( "PhysX3" )
physx3_variation.options.append ( Option ( ["RigidBodyCollection","PhysicsBodyCollection"], "physicsEngine", "PhysX3" ) )

sofa_variation= Variation ( "SOFA" )
sofa_variation.options.append ( Option ( ["RigidBodyCollection","PhysicsBodyCollection"], "physicsEngine", "SOFA" ) )

print ""
print "WARNING: Do not change the window focus, or perform other input until the test is complete!"
print ""
time.sleep(3)

tester= TestExamples( args.output, startup_time= 10, shutdown_time= 5 )
results= tester.testAllExamples( [ode_variation, bullet_variation, physx_variation, physx3_variation, sofa_variation], directory= args.path )

reporter= TestReport()
print reporter.reportResults ( results )

html_reporter= TestReportHTML( args.output )
html_reporter.reportResults ( results )

html_reporter_errors= TestReportHTML( args.output, only_failed= True )
html_reporter_errors.reportResults ( results )

# Exit with error if any errors were detected
sys.exit ( getExitCode ( results ) )
