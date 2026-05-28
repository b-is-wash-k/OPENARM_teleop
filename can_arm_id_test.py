import subprocess
import time

def run(cmd):
    print("\n$", " ".join(cmd))
    return subprocess.run(cmd, text=True)

def test(iface, name):
    print("\n==============================")
    print(f"TESTING: {iface} ({name})")
    print("==============================")

    run([
        "python", "-m", "openarm.damiao", "enable",
        "--motor-type", "DM4340",
        "--iface", iface,
        "3", "19"
    ])

    input("\nPress ENTER to wiggle motor...")

    run([
        "python", "-m", "openarm.damiao", "control", "mit",
        "--motor-type", "DM4340",
        "--iface", iface,
        "3", "19",
        "10.0", "2.0", "0.3", "0.0", "0.0"
    ])

    time.sleep(2)

    run([
        "python", "-m", "openarm.damiao", "enable",
        "--motor-type", "DM4340",
        "--iface", iface,
        "3", "19"
    ])


if __name__ == "__main__":
    print("CAN ↔ ARM IDENTIFICATION TEST")

    test("robot_l", "UNKNOWN ARM")
    test("robot_r", "UNKNOWN ARM")
