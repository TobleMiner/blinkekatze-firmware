#!/usr/bin/env python3

import cv2
import numpy
import struct
import sys

def printe(*args):
	print(*args, file=sys.stderr)

if len(sys.argv) != 5:
	printe(f"Usage: {sys.argv[0]} <max in> <infile> <max out> <outfile>")
	sys.exit(1)

inmax = int(sys.argv[1])
infilename = sys.argv[2]
outmax = int(sys.argv[3])
outfilename = sys.argv[4]

bgr = cv2.imread(infilename, cv2.IMREAD_UNCHANGED).astype(numpy.float32)

height = bgr.shape[0]
width = bgr.shape[1]

bgr *= outmax
bgr /= inmax

with open(outfilename, "wb") as outfile:
	for y in range(height):
		for x in range(width):
			[b, g, r] = bgr[y][x]
			outfile.write(struct.pack('<HHH', round(r), round(g), round(b)))
