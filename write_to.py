import sys
import psutil

with open(sys.argv[1], 'w') as f:
    for p in psutil.process_iter():
        print(f"{p.pid}:{p.name()}", file=f)