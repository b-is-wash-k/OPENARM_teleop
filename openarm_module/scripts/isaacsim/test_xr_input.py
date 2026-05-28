from isaacsim import SimulationApp

launcher_args = {
    "headless": False,
    "experience": "/home/vision/workspace/simlab/.venv-isaacsim/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.base.xr.vr.kit"
}
app = SimulationApp(launcher_args)

import omni.kit.app
import omni.kit.xr.core as xr_core

XR_EXTENSIONS = [
    "omni.kit.xr.core",
    "omni.kit.xr.system.openxr",
    "omni.kit.xr.profile.vr",
    "isaacsim.xr.input_devices",
]

def enable_xr_extensions():
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in XR_EXTENSIONS:
        if not ext_manager.is_extension_enabled(ext):
            ext_manager.set_extension_enabled_immediate(ext, True)

enable_xr_extensions()

frame = 0
xr = None

while app.is_running():
    app.update()
    frame += 1

    if frame % 60 != 0:
        continue

    # Get singleton only after XR session is active
    try:
        if xr is None:
            xr = xr_core.XRCore.get_singleton()

        devices = xr.get_all_input_devices()
        if not devices:
            print("No XR devices found yet - make sure Quest 2 is connected")
            continue

        for device in devices:
            name = device.get_name()
            pose_names = device.get_pose_names()
            print(f"Device: {name}, poses: {pose_names}")
            for pose_name in pose_names:
                pose = device.get_pose(pose_name)
                print(f"  {pose_name}: {pose}")

    except Exception as e:
        print(f"XR error: {e}")

app.close()