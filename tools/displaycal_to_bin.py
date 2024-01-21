#!/usr/bin/env python3

import png
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

reader = png.Reader(infilename)
width, height, pixels, metadata = reader.read_flat()
imagedata = numpy.array(list(map(numpy.double, pixels)))
rgb = numpy.reshape(imagedata, (height, width, 3))

rgb *= outmax
rgb /= inmax

with open(outfilename, "wb") as outfile:
	for y in range(height):
		for x in range(width):
			[r, g, b] = rgb[y][x]
			outfile.write(struct.pack('<HHH', round(r), round(g), round(b)))
