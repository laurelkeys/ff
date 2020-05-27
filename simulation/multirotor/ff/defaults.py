import os


class Default:

    ENV_ROOT_ALIASES = {
        "doc": "D:\\Documents\\AirSim",  # .exe files
        "dev": "D:\\dev\\AirSim\\Unreal\\Environments",  # .sln and .uproject files
        "custom": "D:\\dev\\UE4\\Custom Environments",   # .sln and .uproject files
    }

    ARGS = {
        "airsim_root": "D:\\dev\\AirSim",
        "env_root": ENV_ROOT_ALIASES["custom"],
        "devenv_exe": "C:\\Program Files (x86)\\Microsoft Visual Studio\\2019\\Community\\Common7\\IDE\\devenv.exe",
        "unreal_editor_exe": "D:\\Program Files\\Epic Games\\UE_4.24\\Engine\\Binaries\\Win64\\UE4Editor.exe",
    }

    AIRSIM_CLIENT_PATH = os.path.join(ARGS["airsim_root"], "PythonClient", "airsim")

    SETTINGS_PATH = "D:\\Documents\\AirSim\\settings.json"