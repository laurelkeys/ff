import ff

from wrappers.airsimy import AirSimSettings

try:
    import airsim
except ModuleNotFoundError:
    ff.add_airsim_to_path(airsim_path=ff.Default.AIRSIM_PYCLIENT_PATH)
    import airsim

if __name__ == "__main__":
    settings = AirSimSettings(
        sim_mode=ff.SimMode.Multirotor,
        vehicles=[
            AirSimSettings.Vehicle(
                "Drone1",
                position=airsim.Vector3r(0, -10, -10)
            )
        ]
    )

    settings_dict = settings.as_dict()
    print(settings_dict)
    print(ff.SettingJson.as_str(settings_dict))


