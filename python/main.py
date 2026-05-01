from arduino.app_utils import *
import time

# Command format:
#   l.f.20.750      left forward, 20%, 750 ms
#   l.b.40.1000     left backward, 40%, 1000 ms
#   r.f.30.500      right forward, 30%, 500 ms
#   all.f.50.1200   both forward, 50%, 1200 ms
#   all.s           stop both
#
# Percent sign is allowed:
#   l.f.20%.750

def send(cmd: str):
    cmd = cmd.strip().lower().replace(" ", "").replace("%", "")

    print(f"CMD: {cmd}")

    if cmd in ("stop", "x", "all.s", "a.s"):
        Bridge.call("drive", 2, 0, 0, 0)
        return

    parts = cmd.split(".")

    if len(parts) < 2:
        print("Bad command. Example: l.f.20.750")
        return

    motor_txt = parts[0]
    dir_txt = parts[1]

    if motor_txt in ("l", "left"):
        motor_code = 0
    elif motor_txt in ("r", "right"):
        motor_code = 1
    elif motor_txt in ("a", "all", "both"):
        motor_code = 2
    else:
        print("Bad motor. Use l, r, or all.")
        return

    if dir_txt in ("f", "forward"):
        dir_code = 1
    elif dir_txt in ("b", "back", "backward"):
        dir_code = -1
    elif dir_txt in ("s", "stop"):
        Bridge.call("drive", motor_code, 0, 0, 0)
        return
    else:
        print("Bad direction. Use f, b, or s.")
        return

    power = 100
    duration_ms = 0

    if len(parts) >= 3 and parts[2] != "":
        power = int(parts[2])

    if len(parts) >= 4 and parts[3] != "":
        duration_ms = int(parts[3])

    power = max(0, min(100, power))
    duration_ms = max(0, duration_ms)

    Bridge.call("drive", motor_code, dir_code, power, duration_ms)


def wiggle_demo():
    send("l.f.60.500")
    time.sleep(0.8)

    send("l.b.60.500")
    time.sleep(0.8)

    send("r.f.60.500")
    time.sleep(0.8)

    send("r.b.60.500")
    time.sleep(0.8)

    send("all.s")


def loop():
    print("Bun command console ready.")
    print("Type commands like:")
    print("  l.f.60.500")
    print("  l.b.60.500")
    print("  r.f.60.500")
    print("  r.b.60.500")
    print("  all.f.50.1000")
    print("  all.s")
    print("Type q to quit.")

    while True:
        cmd = input("bun> ").strip()

        if cmd.lower() in ("q", "quit", "exit"):
            send("all.s")
            print("Stopped motors. Exiting.")
            break

        if cmd:
            send(cmd)


App.run(user_loop=loop)