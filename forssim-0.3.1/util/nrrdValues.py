import os
cmd = "unu histo -b 256 -i " + os.sys.argv[1] + " -o temp.txt"
os.system(cmd)
f = file("temp.txt")

print os.sys.argv[1], "has values (number of voxes with each value):"
vsum = 0
for i in range(256) :
  v = int(f.readline())
  if v is not 0:
    print i, "\t", v
  vsum += v
print "----------------------"
print "Sum \t", vsum
print

os.system("rm temp.txt")


