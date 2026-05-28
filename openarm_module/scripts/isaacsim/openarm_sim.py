from isaaclab.app import AppLauncher

launcher_args = {
    "headless": False,
    "experience": "/home/vision/workspace/simlab/.venv-isaacsim/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.base.xr.vr.kit"
}

app_launcher = AppLauncher(launcher_args)
simulation_app = app_launcher.app

import omni.kit.app
import isaaclab.sim as sim_utils

BIMANUAL_USD = "/home/vision/humanoids/openarm_isaac_lab/source/openarm/openarm/tasks/manager_based/openarm_manipulation/usds/openarm_bimanual/openarm_bimanual.usd"

XR_EXTENSIONS = [
    "omni.kit.xr.core",
    "omni.kit.xr.system.openxr",
    "omni.kit.xr.profile.vr",
]

def enable_xr_extensions():
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in XR_EXTENSIONS:
        if not ext_manager.is_extension_enabled(ext):
            print(f"Enabling {ext}...")
            ext_manager.set_extension_enabled_immediate(ext, True)
        else:
            print(f"{ext} already enabled.")

def main():
    sim_cfg = sim_utils.SimulationCfg(dt=0.01)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 2.0, 2.0], [0.0, 0.0, 0.5])

    cfg = sim_utils.UsdFileCfg(usd_path=BIMANUAL_USD)
    cfg.func("/World/OpenArm", cfg)

    enable_xr_extensions()

    sim.reset()
    print("Bimanual OpenArm loaded with VR extensions enabled.")

    while simulation_app.is_running():
        sim.step()

if __name__ == "__main__":
    main()
    simulation_app.close()