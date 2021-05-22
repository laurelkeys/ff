# This is just a dump of the hacky code used directly in the REPL to convert
# the alignment matrix data from MeshLab's .mlp file to a NumPy array, copy:
from sys import argv
from xml.etree import ElementTree as ET
x = argv[1]
for p in ET.parse(x).getroot().findall(".//MLMatrix44"): print(f"[{','.join(p.text.split(' '))}]")
