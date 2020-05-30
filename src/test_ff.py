import ff
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    ff.add_arguments_to(parser)
    args = parser.parse_args()
    ff.launch_env_from_args(args)