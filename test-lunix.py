#########################################
#
#  Chris Markou <chrs.markx86@gmail.com>
#
#########################################

# A simple script to test the implementation 
# of our driver in a user-friendly way.

#!/usr/bin/env python
import re
import os

print "Welcome to Lunix-TNG driver program!\n"

print "Which sensor do you choose? [0-15]",
sensor = raw_input()
print "What measurement do you need? [temp/batt/light]",
msr = raw_input()

print "\nThat's fine, press CTR+C if you change mind!"
print "LunixSensor_" + str(sensor) + "_" + str(msr) +":"

exec_var = "cat /dev/lunix" + str(sensor) + "-" + str(msr)
os.system(exec_var)

