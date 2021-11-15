#!usr/bin/env python3

import sys
import subprocess

# add necessary packages to the packages array
packages = ['opencv-contrib-python', 'argparse', 'numpy', 'imutils']

# implement pip as a subprocess:
subprocess.check_call([sys.executable, '-m', 'pip', 'upgrade', 'pip'])
for p in packages:
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', p])
