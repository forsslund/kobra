#!/usr/bin/python

## Modify header files to include example files in doxygen comments.
## Outputs a list of example files that do not exist in any headers.
## Will output coordaxes.x3d since it is inlined in other x3d files.

import os, sys
import re
from os.path import join

current_dir = os.getcwd()
examples_dirs = ["../examples/RigidBody", "../examples/softbody"]
examples = []
ignoreList = ["BulletJoints", "BulletCallbacks", "PHYSXCallbacks", "ODECallbacks", "PhysicsEngineParameters", "PhysicsEngineThread", "cooking", "FieldTemplates", "PythonMethods"]

for examples_dir in examples_dirs:
  if os.path.isdir( examples_dir ):
    temp_examples = []
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
          temp_examples.append( temp_file.replace( ".x3d", "" ) )
    examples.append( ( examples_dir[examples_dir.rfind("/") + 1:len( examples_dir ) ], temp_examples ) )
  else:
    print "The examples/All directory does not exist, this path was used: " + examples_dir

headers_dir = current_dir + "/../include/H3D/H3DPhysics"
if os.path.isdir( headers_dir ):
  print "Handling headers directory " + headers_dir

  print "ALSJSSJLSLWJLJL"
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
      if( not temp_file.startswith("X3D") and not temp_file.startswith("H3D") and temp_file.endswith(".h") ):
        file_name_no_end = temp_file.replace( ".h", "" )
        if file_name_no_end in ignoreList:
          continue
        name_count = 0
        directory_prefix = ""
        for temp_examples in examples:
          tmp_name_count = temp_examples[1].count( file_name_no_end )
          if tmp_name_count > 0:
            directory_prefix = temp_examples[0]
          name_count = name_count + tmp_name_count
        if name_count > 0:
          # name count should not exceed 1
          if( name_count > 1 ):
            print "name_count is above 1, there are several examples to choose from for node ", file_name_no_end
            continue

          fhandle = open( join( headers_dir, temp_file ), "rb" )
          code = fhandle.read()
          fhandle.close()

          new_string1 = """  ///   - <a href="../../examples/%s/%s.x3d">%s.x3d</a>""" %( directory_prefix, file_name_no_end, file_name_no_end )
          new_string2 = """  ///     ( <a href="examples/%s.x3d.html">""" % ( file_name_no_end )

          if code.find( new_string1 ) == -1 and code.find( new_string2 ) == -1:
            old_string1 = """/// \par Internal routes:"""
            new_string = """///
  /// <b>Examples:</b>
""" + new_string1 + "\n" + new_string2 + "Source</a> )\n  "
            print new_string
            new_code = code.replace( old_string1, new_string + "///\n  " + old_string1, 1 )
            print """Code replaced for %s.h""" % (file_name_no_end)

            if new_code != code:
              print "Change code for %s.h" % (file_name_no_end)
              fhandle = open( join( headers_dir, temp_file ), "wb" )
              fhandle.write( new_code )
              fhandle.close()
              for i in range( len( examples ) ):
                if file_name_no_end in examples[i][1]:
                  examples[i][1].remove( file_name_no_end )
          else:
            for i in range( len( examples ) ):
              if file_name_no_end in examples[i][1]:
                examples[i][1].remove( file_name_no_end )

    for temp_file in files:
      if( not temp_file.startswith("X3D") and not temp_file.startswith("H3D") and temp_file.endswith(".h") ):
        fhandle = open( join( headers_dir, temp_file ), "rb" )
        code = fhandle.read()
        fhandle.close()
        examples_to_remove = []
        for tmp_examples in examples:
          for i in tmp_examples[1]:
            find_string1 = """  ///   - <a href="../../examples/%s/%s.x3d">%s.x3d</a>""" % (tmp_examples[0],i, i)
            find_string2 = """  ///     ( <a href="examples/%s.x3d.html">""" % (i)
            if code.find( find_string1 ) != -1 and code.find( find_string2 ) != -1:
              examples_to_remove.append( i )

        for i in examples_to_remove:
          for j in range( len( examples ) ):
            if i in examples[j][1]:
              examples[j][1].remove( i )

    print "These are the example files that are not assigned in the documentation"
    for tmp_examples in examples:
      for i in tmp_examples[1]:
        print i + ".x3d"
else:
  print "The directory for H3D headers does not exist, this path was used: " + headers_dir
