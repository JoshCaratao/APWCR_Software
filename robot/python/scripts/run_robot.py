from pathlib import Path
import sys

# Add robot/python to sys.path so 'pwc_robot' can be imported when running this script directly
THIS_DIR = Path(__file__).resolve().parent
PY_ROOT = THIS_DIR.parent  # robot/python
sys.path.insert(0, str(PY_ROOT))

from pwc_robot.main import main

import socket

def _print_lan_ips() -> None:
    ips = set()
    host = socket.gethostname()
    for info in socket.getaddrinfo(host, None, family=socket.AF_INET):
        ip = info[4][0]
        if not ip.startswith("127.") and not ip.startswith("169.254."):
            ips.add(ip)

    if ips:
        print("LAN IP(s):")
        for ip in sorted(ips):
            print(f"  http://{ip}:5000")
    else:
        print("LAN IP not found (check network connection).")
    print()


if __name__ == "__main__":
    _print_lan_ips()
    # Change this if you want a different config file
    main(config_name="robot_default.yaml")
