import ff
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    ff.add_arguments_to(parser)
    args = parser.parse_args()
    print(args)
    print(ff.LaunchEnvArgs(args))
    # print(*ff.LaunchEnvArgs(args))