#! /usr/bin/python

import sys
import glob
from jsfview import find_msg_types

for filename in glob.iglob(sys.argv[1] + '/**/*.jsf', recursive=True):
    msg_types = find_msg_types(filename)
    if 3000 in msg_types:
        print(filename)
    #print(sorted(msg_types))
