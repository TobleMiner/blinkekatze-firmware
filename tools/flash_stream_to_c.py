#!/usr/bin/env python3

import sys

def printe(*args):
	print(*args, file=sys.stderr)

if len(sys.argv) != 4:
	printe(f"Usage: {sys.argv[0]} <infile> <outfile> <symbol name>")
	sys.exit(1)

HEADER = '''#include "bq27546.h"

'''

infilename = sys.argv[1]
outfilename = sys.argv[2]
symbolname = sys.argv[3]

with open(infilename, "r") as infile:
	with open(outfilename, "w") as outfile:
		commanddefs = []
		datacnt = 0
		outfile.write(HEADER)
		for line in infile:
			if line.startswith(";"):
				pass
			elif line.startswith("W:"):
				parts = line.split(" ")
				address = int(parts[1], 16) >> 1
				data = map(lambda x: f"0x{int(x, 16):02x}", parts[2:])
				outfile.write(f"static const uint8_t data_{datacnt}[] = {{ {', '.join(data)} }};\n")
				commanddefs.append(f"{{ BQ27546_FLASH_CMD_WRITE, .write = {{ 0x{address:02x}, data_{datacnt}, sizeof(data_{datacnt}) }} }}")
				datacnt += 1
			elif line.startswith("C:"):
				parts = line.split(" ")
				address = int(parts[1], 16) >> 1
				register = int(parts[2], 16)
				data = map(lambda x: f"0x{int(x, 16):02x}", parts[3:])
				outfile.write(f"static const uint8_t data_{datacnt}[] = {{ {', '.join(data)} }};\n")
				commanddefs.append(f"{{ BQ27546_FLASH_CMD_COMPARE, .compare = {{ 0x{address:02x}, 0x{register:02x}, data_{datacnt}, sizeof(data_{datacnt}) }} }}")
				datacnt += 1
			elif line.startswith("X:"):
				parts = line.split(" ")
				delayms = int(parts[1], 10)
				commanddefs.append(f"{{ BQ27546_FLASH_CMD_WAIT, .wait = {{ {delayms} }} }}")
			else:
				raise Exception(f"Can not parse line \"{line}\"")
		outfile.write(f'''
const bq27546_flash_op_t {symbolname}[] = {{
''')
		outfile.write(",\n".join(map(lambda x: f"\t{x}", commanddefs)))
		outfile.write(f'''
}};

unsigned int {symbolname}_num_ops = sizeof({symbolname}) / sizeof(*{symbolname});
''')

