""" Contains the Process interface definition as well as implementations of it for Unix and Windows """

import os
import sys
import subprocess
import time
import platform
if sys.version_info[0] >= 3:
  from queue import Queue, Empty
else:
  from Queue import Queue, Empty
from threading  import Thread
import tempfile
import string
import re
import argparse
import math
import signal

if platform.system() == 'Windows':
  import win32api
  import win32com.client
  import win32gui
  import win32con
  import win32process
  import psutil
  import wmi



class Process(object):
  """ Interface for manipulating a process. """

  def launch(self, command, cwd):
    pass

  def testLaunch(self, command, cwd, startup_time, shutdown_time, startup_time_multiplier, early_shutdown_file):
    """ Makes sure early_shutdown_file doesn't exist, calls launch() and then waits for the specified startup and shutdown time. """
    pass
    
  def isRunning(self):
    """ Checks if the process associated with this object is running. """
    pass
    
  def sendKey(self, key):
    pass
    
  def kill(self):
    pass
    
  def getStdOut(self):
    pass
    
  def getStdErr(self):
    pass
    


class ProcessWin32(Process):

  def launch(self, command, cwd):
  
    def enqueue_output(stdout, stderr, std_out_q, std_err_q):
      for line in iter(stdout.readline, b''):
        std_out_q.put(line)
      stdout.close()
      for line in iter(stderr.readline, b''):
        std_err_q.put(line)
      stderr.close()
  
    self.process_name = command[0]
  
    shell = win32com.client.Dispatch("WScript.Shell")

    import ctypes
    ctypes.windll.kernel32.SetErrorMode(0)
    print("launching: " + " ".join(command))
    self.process = subprocess.Popen(" ".join(command), shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd= cwd)
    
    self.std_out_q = Queue()
    self.std_err_q = Queue()
    self.monitor_output_thread = Thread(target= enqueue_output, args=(self.process.stdout, self.process.stderr, self.std_out_q, self.std_err_q))
    self.monitor_output_thread.start()
    
  def testLaunch(self, command, cwd, startup_time, shutdown_time, startup_time_multiplier, early_shutdown_file):
  
    import ctypes
    SEM_NOGPFAULTERRORBOX = 0x0002 # From MSDN
    ctypes.windll.kernel32.SetErrorMode(SEM_NOGPFAULTERRORBOX)
    subprocess_flags = 0x8000000 #win32con.CREATE_NO_WINDOW?
    
    self.process_name = command[0]
    
    if os.path.isfile(early_shutdown_file):
      os.remove(early_shutdown_file)
    self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd= cwd, creationflags= subprocess_flags)
    for i in range(0, startup_time_multiplier):
      time.sleep(startup_time)
      if os.path.isfile(early_shutdown_file) or not self.isRunning():
        break
    success = self.isRunning()
    if success:
      self.kill()
    return success
    
  def isRunning(self):
    # for Windows XP use this
    try:
      return psutil.pid_exists(self.process.pid)
    except:
      return False

  def sendKey(self, key):
    if key == "{ESC}":
      # This will allow us to send ESC without the window being in focus
      target_pid = self.process.pid
      h3d_handle = 0
      
      def win32_enum_handles_callback(handle, dummy):
        _, found_pid = win32process.GetWindowThreadProcessId(handle)
        if found_pid == target_pid:
          h3d_handle = handle
        return True
      
      try:
        win32gui.EnumWindows(win32_enum_handles_callback, h3d_handle)
        win32api.PostMessage(h3d_handle, 
            win32con.WM_KEYDOWN, 
            win32con.VK_ESCAPE, 
            0)
      except Exception as e:
        print(e)
    else:
      shell = win32com.client.Dispatch("WScript.Shell")
      shell.AppActivate(self.process_name)
      time.sleep(0.1)
      shell.SendKeys(key)

  
  def killProcessThroughWMI(self, process):
    # function to kill process through python wmi package
    # the input process can be either process name or process id

    processByName = not isinstance( process, int )
    c = wmi.WMI()
    processes = []
    if processByName:
      processes = c.Win32_Process (name=str(process))
    else:
      processes = c.Win32_Process (ProcessId=int(process))
    for p in processes:
      
      try:
        # if check is done in case process is terminated due to any other reason already 
        # this looks wired as it was just tested before in the for loop, but it indeed can avoid
        # some unnecessary process killing which might be related with a potential bug in Win32_Process
        # function , seems the process just get terminated are still recognizable until it is checked
        # multiple times with Win32_Process function
        if len( c.Win32_Process ( ProcessId=int(process) ) ) > 0:
          print( "Killing " + str( p.ProcessId )+ " " + str(p.Name) )
          result = p.Terminate()[0]
          if result == 0:
            print( "Killing " + str( p.ProcessId ) + " " + str( p.Name ) + " succeeded!" )
          else:
            print( "Killing " + str( p.ProcessId ) + " " + str( p.Name ) + " failed!" )
            if result == 2:
              print( "Failed due to access denied" )
            elif result == 3:
              print( "Failed due to insufficient privilege" )
            elif result == 8:
              print( "Failed due to unknown failure" )
            elif result == 9:
              print( "Failed due to path not found" )
            elif result == 21:
              print( "Failed due to invalid parameter" )
            else:
              print( "Failed due to unknown reason" )
        else:
          print( "Process ", process, " no longer exist, no need to kill again." )
      except:
        print( "Something went wrong while trying to kill" + str( p.ProcessId ) + " " + str( p.Name ) )


  def kill(self):

    # If the process crashed then windows might have started WerFault.exe that
    # might be preventing the process from properly shutting down.
    # We can solve this by looking for any WerFault process that was launched
    # with the process id of our process and shut that down.
    # Then if H3D is still running it means werfault wasn't keeping it alive
    # and we can go ahead and do the taskkill.
 
    for proc in psutil.process_iter():
      if "WerFault.exe".lower() in proc.name().lower():
        # NOTE: Blanket kill all WerFault instances!
        print("killing werfault.exe")
        proc.kill()
        
        # # Check if WerFault is for our process
        # cmdparams = proc.cmdline()
        # for i in range(0, len(cmdparams)):
          # if (cmdparams[i] == "-p") and (i < len(cmdparams)-1) and (cmdparams[i+1] == str(self.process.pid)):
            # #This is the WerFault.exe that is forcefully keeping our process alive!
            # proc.kill()
        
    try:
      psutil_proc = psutil.Process(self.process.pid)
      for proc_child in psutil_proc.children(recursive=True):
        print("killing children process, ", proc_child.name(), " with pid ", proc_child.pid)
        self.killProcessThroughWMI(proc_child.pid)
      
      if self.isRunning():
        print("killing main process:")
        self.killProcessThroughWMI(psutil_proc.pid)

    except:
      pass



  def getStdOut(self):
    output = ""
    try:
      while not self.std_out_q.empty():
        output+= self.std_out_q.get(timeout=.1).decode("utf-8")
    except Empty:
      pass
    return output
    
  def getStdErr(self):
    output = ""
    try:
      while not self.std_err_q.empty():
        output+= self.std_err_q.get(timeout=.1).decode("utf-8")
    except Empty:
      pass
    return output



class ProcessUnix(Process):
  def launch(self, command, cwd):
      
    def enqueue_output(stdout, stderr, std_out_q, std_err_q):
      for line in iter(stdout.readline, b''):
        std_out_q.put(line)
      stdout.close()
      for line in iter(stderr.readline, b''):
        std_err_q.put(line)
      stderr.close()
  
    self.process_name = command[0]

    self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd= cwd)
    
    self.std_out_q = Queue()
    self.std_err_q = Queue()
    self.monitor_output_thread = Thread(target= enqueue_output, args=(self.process.stdout, self.process.stderr, self.std_out_q, self.std_err_q))
    self.monitor_output_thread.start()
    
  def testLaunch(self, command, cwd, startup_time, shutdown_time, startup_time_multiplier, early_shutdown_file):
  
    self.process_name = command[0]
    if os.path.isfile(early_shutdown_file):
      os.remove(early_shutdown_file)
    self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd= cwd)
    for i in range(0, startup_time_multiplier):
      time.sleep(startup_time)
      if os.path.isfile(early_shutdown_file) or not self.isRunning():
        break
    success = self.isRunning()
    if success:
      self.kill()
      time.sleep(shutdown_time)
    return success
    
  def isRunning(self):
    #for Linux use this
    query_process = 'ps -Af | grep ' + self.process_name
    
    p = subprocess.Popen(query_process, stdout=subprocess.PIPE, shell=True)
    output = p.communicate()
    output0 = output[0]
    # remove ps and grep process lines.
    output0 = output0.replace('ps -Af | grep ' + self.process_name, "")
    output0 = output0.replace('grep ' + self.process_name, "")
    return output0.find(self.process_name) >= 0 and not (output0.find('[' + self.process_name + '] <defunct>') >= 0)
    
  def sendKey(self, key):
    time.sleep(0.1)
    if key == "{ESC}" or key == "{Esc}":
#      device = uinput.Device([uinput.KEY_ESC])
#      device.emit_click(uinput.KEY_ESC)
      key_process = 'xte "key Escape"'
      p = subprocess.Popen(key_process, stdout=subprocess.PIPE, shell=True)
    else:
      raise Exception("Key " + key + " not handled exception")
    
  def kill(self):
    self.process.kill()
    
  def getStdOut(self):
    output = ""
    try:
      while not self.std_out_q.empty():
        output+= self.std_out_q.get(timeout=.1)
    except Empty:
      pass
    return output
    
  def getStdErr(self):
    output = ""
    try:
      while not self.std_err_q.empty():
        output+= self.std_err_q.get(timeout=.1)
    except Empty:
      pass
    return output
