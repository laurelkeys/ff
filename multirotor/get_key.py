from sys import argv
from msvcrt import getch

def get_key():
    first_char = getch()
    if first_char == b'\xe0':
        char = chr(getch()[0])
        key_val = {'H': "up", 'P': "down", 'M': "right", 'K': "left"}[char]
    elif first_char in [b'\x1b', b'\x03', b'\x04']:
        # [Esc, Ctrl+C, Ctrl+D]
        key_val = False
    else:
        key_val = chr(ord(first_char))
    return key_val, first_char

i = 10 if len(argv) < 2 else int(argv[1])
while i > 0:
    key_val, _ = get_key()
    if key_val == False:
        print("Quitting")
        exit()
    print(key_val)
    i -= 1