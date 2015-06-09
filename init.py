#!/usr/bin/env python
import re
import os
import time

print " Initializing sensors' attachment ..."

os.system("./lunix_dev_nodes.sh")
os.system("./lunix-tcp.sh /dev/ttyS0 &")
os.system("pwd")
print "Wait for cerberus..."
time.sleep( 5 )
os.system("./lunix-attach /dev/ttyS0 &")

