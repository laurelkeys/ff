import os, sys

__vendor_path = os.path.abspath(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "vendor"
    )
)

sys.path.insert(0, __vendor_path)

del sys, os

if __name__ == "__main__":
    print(f"vendor path = '{__vendor_path}'")
