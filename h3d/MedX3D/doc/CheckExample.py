#!/usr/bin/python

##  This file does a few things:
##  1. Finds H3DAPI nodes that do not have a corresponding example file added to examples/All.
##  2. Finds any example files (.x3d) in examples/All that are not explicitly referenced in a header.
##  
##  NOTE: There's a long list of ignored nodes that don't have an example, 
##  typically because they are core nodes that don't quite produce anything "example worthy".

import os, sys
import re
from os.path import join

current_dir = os.getcwd()
examples_dir = "../x3d"
examples = []
examples_duplicate = dict() # Used when finding unmentioned example files by removing every entry that is mentioned in a header file.
count = 0
ignoreList = ["Integrator", "MedX3D", "VolumeGradient"]
pendingList = []
example_files_ignore_list = []

if os.path.isdir( examples_dir ):
  print "Handling examples directory " + examples_dir

  for root, dirs, files in os.walk(examples_dir):
    if '.svn' in dirs:
      dirs.remove('.svn')  # don't visit CVS directories
    if 'branches' in dirs:
      dirs.remove('branches')  # don't visit branches directories
    if 'obj' in dirs:
      dirs.remove('obj')  # don't visit obj directories
    if 'vc7' in dirs:
      dirs.remove('vc7')  # don't visit vc7 directories
    if 'vc8' in dirs:
      dirs.remove('vc8')  # don't visit vc8 directories
    if 'vc9' in dirs:
      dirs.remove('vc9')  # don't visit vc9 directories
    for temp_file in files:
      if( temp_file.endswith(".x3d") ):
        examples_duplicate[temp_file] = False
        examples.append( temp_file.replace( ".x3d", "" ) )
else:
  print "The examples/All directory does not exist, this path was used: " + examples_dir

headers_dir = current_dir + "/../include/H3D/MedX3D"
if os.path.isdir( headers_dir ):
  print "Handling headers directory " + headers_dir

  for root, dirs, files in os.walk(headers_dir):
    if '.svn' in dirs:
      dirs.remove('.svn')  # don't visit CVS directories
    if 'branches' in dirs:
      dirs.remove('branches')  # don't visit branches directories
    if 'obj' in dirs:
      dirs.remove('obj')  # don't visit obj directories
    if 'vc7' in dirs:
      dirs.remove('vc7')  # don't visit vc7 directories
    if 'vc8' in dirs:
      dirs.remove('vc8')  # don't visit vc8 directories
    if 'vc9' in dirs:
      dirs.remove('v bc9')  # don't visit vc9 directories
    for temp_file in files:
      # If the file is a header ...
      if( not temp_file.startswith("X3D") and not temp_file.startswith("H3D") and temp_file.endswith(".h") ):
        file_name_no_end = temp_file.replace( ".h", "" )
        name_count = examples.count( file_name_no_end )
        if file_name_no_end not in ignoreList and file_name_no_end not in pendingList:
          fhandle = open( join( headers_dir, temp_file ), "rb" )
          code = fhandle.read()
          fhandle.close()
          # See if the header contains any mentions of the examples folder.
          example_location_string = """  ///   - <a href="../../x3d"""
          if code.find( example_location_string ) == -1:
            print "This is a file with no example: " + file_name_no_end
            count += 1
          
          # See if any of the example files are mentioned in the header.
          for example_file in examples_duplicate.iterkeys():
            if code.find( example_file ) != -1:
              examples_duplicate[example_file] = True
          
        elif name_count > 1:
          # name count should not exceed 1
          print "name_count is above 1, there should not be two files with the same name in the MedX3D/x3d directory"

for k, v in examples_duplicate.items():
  if v is False and k.replace( ".x3d", "" ) not in example_files_ignore_list:
    print  "This is an example file that is not mentioned in any header: " + k
print "This is the total number of files: ", count