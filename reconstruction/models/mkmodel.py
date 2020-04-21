import os
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Setup the folder structure for a new model"
    )

    parser.add_argument("model_name", help="Name of the model (to be used as the folder name).")
    parser.add_argument("--current_dir", action="store_true", help="Create folders in the current dir, instead of in the script's dir.")

    args = parser.parse_args()

    try:
        folder_name = args.model_name
        if not args.current_dir:
            folder_name = os.path.join(os.path.dirname(os.path.realpath(__file__)), folder_name)

        os.makedirs(folder_name)
        os.makedirs(os.path.join(folder_name, "data"))
        os.makedirs(os.path.join(folder_name, "images"))
        os.makedirs(os.path.join(folder_name, "Meshroom"))
        os.makedirs(os.path.join(folder_name, "Meshroom", "publish"))

        # with open(os.path.join(folder_name, ".gitignore"), 'w') as f:
        #     f.write('\n'.join([
        #         "images/",
        #         "data/*.ply",
        #         "data/*.mlp",
        #         "Meshroom/*.mg",
        #         "Meshroom/MeshroomCache/",
        #     ]))

    except FileExistsError:
        print(f"WARNING: Directory '{args.model_name}' already exists (execution aborted).")