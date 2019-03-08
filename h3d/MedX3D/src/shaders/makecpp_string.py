# A simple pythonscript that transforms the .glsl file to a cpp string that can be used in X3DVolumeNode.cpp
# Use it to convert from development script to hard-code it easily.
import os

file_to_read = "StyleFunctions.glsl"
file_to_write = "StyleFunctions_cppstring.glsl"
result_string = ""
print "reading file " + file_to_read
fhandle = open( file_to_read, "r" )
data = fhandle.readlines()
print len(data)
fhandle.close()
print "file reading done"

for i in range(len(data )):
  data[i] = data[i].rstrip("\n")
  data[i] = "\"" + data[i] + "\\n\"\n"
  print data[i]

print "writing file " + file_to_write
fhandle = open( file_to_write, "w" )
fhandle.writelines( data )
fhandle.close()
print "file writing done"

file_to_read = "Slices_FS_main.glsl"
file_to_write = "Slices_FS_main_cppstring.glsl"
result_string = ""
print "reading file " + file_to_read
fhandle = open( file_to_read, "r" )
data = fhandle.readlines()
print len(data)
fhandle.close()
print "file reading done"

for i in range(len(data )):
  data[i] = data[i].rstrip("\n")
  data[i] = "\"" + data[i] + "\\n\"\n"
  print data[i]

print "writing file " + file_to_write
fhandle = open( file_to_write, "w" )
fhandle.writelines( data )
fhandle.close()
print "file writing done"

file_to_read = "RayCaster_FS_main.glsl"
file_to_write = "RayCaster_FS_main_cppstring.glsl"
result_string = ""
print "reading file " + file_to_read
fhandle = open( file_to_read, "r" )
data = fhandle.readlines()
print len(data)
fhandle.close()
print "file reading done"

for i in range(len(data )):
  data[i] = data[i].rstrip("\n")
  data[i] = "\"" + data[i] + "\\n\"\n"
  print data[i]

print "writing file " + file_to_write
fhandle = open( file_to_write, "w" )
fhandle.writelines( data )
fhandle.close()
print "file writing done"
