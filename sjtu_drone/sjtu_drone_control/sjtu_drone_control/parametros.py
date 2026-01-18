#!/usr/bin/env python3
import subprocess
import yaml
import itertools
import time
import os
import signal
import pandas as pd
from datetime import datetime

YAML_PATH = "/ros2_ws/src/sjtu_drone/sjtu_drone_bringup/config/drone.yaml"

BASE_MASTER_PORT = 11345
PORT_SPAN = 50

CSV_PATH = "trajectory_errors.csv"
RUN_SECONDS = 45

# =========================
# FASE 2: barrer vel_xy (P,D)
# roll/pitch fijo al mejor (FASE 1)
# =========================

BEST_ROLL_P = 8.0
BEST_ROLL_D = 6.0

# Barrido (pon aquí los rangos que quieras)
vel_xy_p_vals = [ 4.0, 5.0, 6.0]
vel_xy_d_vals = [ 2.0,2.1   ,2.3,2.5]

# (Opcional) fija positionXY P mientras tuneas velXY
FIX_POS_XY_P = 1.1

LOG_PATH = "tuning_runs.log"


def log_line(msg: str):
    ts = datetime.now().isoformat(timespec="seconds")
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(f"[{ts}] {msg}\n")
        f.flush()


def cleanup_followers():
    subprocess.run(
        ["bash", "-lc",
         "pkill -9 -f trajectory_follower || true; "
         "pkill -9 -f trajectory_follower_quintic_open_endzero || true"
         ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False
    )
    for _ in range(50):
        r = subprocess.run(
            ["bash", "-lc", "pgrep -f trajectory_follower >/dev/null"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        if r.returncode != 0:
            break
        time.sleep(0.1)
    time.sleep(0.2)


def make_ros_env_cmd(cmd: str, master_port: int) -> list:
    return [
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash && "
        "source /ros2_ws/install/setup.bash && "
        f"export GAZEBO_MASTER_URI=http://127.0.0.1:{master_port} && "
        f"{cmd}"
    ]


def make_follower_cmd(master_port: int, run_id: int) -> list:
    return make_ros_env_cmd(
        f"ros2 run sjtu_drone_control trajectory_follower "
        f"--ros-args -r __node:=trajectory_follower_tune_{run_id}",
        master_port
    )


def terminate_process_hard(proc, timeout=5):
    if proc is None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=timeout)
    except Exception:
        try:
            proc.kill()
            proc.wait(timeout=timeout)
        except Exception:
            pass


def send_takeoff(master_port: int, topic: str, retries: int = 10, wait_s: float = 0.5) -> bool:
    for _ in range(retries):
        r = subprocess.run(
            make_ros_env_cmd(f"ros2 topic list | grep -q '^{topic}$'", master_port),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        if r.returncode == 0:
            break
        time.sleep(wait_s)
    else:
        print(f"[WARN] No aparece el tópico {topic}", flush=True)
        return False

    for _ in range(retries):
        cmd = f"ros2 topic info {topic} -v | grep -q 'Subscription count: [1-9]'"
        r = subprocess.run(
            make_ros_env_cmd(cmd, master_port),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        if r.returncode == 0:
            break
        time.sleep(wait_s)

    for _ in range(8):
        subprocess.run(
            make_ros_env_cmd(f"ros2 topic pub -1 {topic} std_msgs/msg/Empty '{{}}'", master_port),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False
        )
        time.sleep(0.2)

    return True


def cleanup_gazebo():
    subprocess.run(
        ["bash", "-lc",
         "pkill -9 -x gzserver || true; "
         "pkill -9 -x gzclient || true; "
         "pkill -9 -f gazebo || true; "
         "pkill -9 -f robot_state_publisher || true; "
         "rm -f /tmp/gazebo* /tmp/.gazebo* 2>/dev/null || true"
         ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False
    )
    for _ in range(50):
        r = subprocess.run(
            ["bash", "-lc", "pgrep -x gzserver >/dev/null || pgrep -x gzclient >/dev/null"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        if r.returncode != 0:
            break
        time.sleep(0.1)
    time.sleep(0.5)


def write_and_verify_yaml(vel_xy_p: float, vel_xy_d: float):
    print("\n[DEBUG] YAML_PATH =", YAML_PATH, flush=True)

    with open(YAML_PATH, "r") as f:
        data = yaml.safe_load(f)

    target = {
        "rollpitchProportionalGain": float(BEST_ROLL_P),
        "rollpitchDifferentialGain": float(BEST_ROLL_D),
        "velocityXYProportionalGain": float(vel_xy_p),
        "velocityXYDifferentialGain": float(vel_xy_d),
        "positionXYProportionalGain": float(FIX_POS_XY_P),
    }

    for k, v in target.items():
        data[k] = v

    print("[DEBUG] Writing gains:",
          data["rollpitchProportionalGain"],
          data["rollpitchDifferentialGain"],
          data["velocityXYProportionalGain"],
          data["velocityXYDifferentialGain"],
          data["positionXYProportionalGain"],
          flush=True)

    with open(YAML_PATH, "w") as f:
        yaml.safe_dump(data, f)

    os.sync()

    with open(YAML_PATH, "r") as f:
        check = yaml.safe_load(f)

    print("[DEBUG] Readback gains:",
          check.get("rollpitchProportionalGain"),
          check.get("rollpitchDifferentialGain"),
          check.get("velocityXYProportionalGain"),
          check.get("velocityXYDifferentialGain"),
          check.get("positionXYProportionalGain"),
          flush=True)

    eps = 1e-12
    for k, v in target.items():
        if k not in check:
            raise RuntimeError(f"YAML verificación fallida: falta clave '{k}'")
        if abs(float(check[k]) - float(v)) > eps:
            raise RuntimeError(f"YAML NO actualizado en '{k}': esperado {v} pero leído {check[k]}")

    print("[DEBUG] YAML verificado correctamente.\n", flush=True)


def run_sim_and_cost(vel_xy_p: float, vel_xy_d: float, run_id: int) -> float:
    master_port = BASE_MASTER_PORT + (run_id % PORT_SPAN)

    print(
        f"\n[TEST #{run_id}] vel_xy_p={vel_xy_p} vel_xy_d={vel_xy_d} | "
        f"roll_p={BEST_ROLL_P} roll_d={BEST_ROLL_D} | "
        f"pos_xy_p={FIX_POS_XY_P} | "
        f"GAZEBO_MASTER_URI=:{master_port}",
        flush=True
    )

    log_line(
        f"RUN_START run_id={run_id} roll_p={BEST_ROLL_P} roll_d={BEST_ROLL_D} "
        f"vel_xy_p={vel_xy_p} vel_xy_d={vel_xy_d} pos_xy_p={FIX_POS_XY_P} "
        f"master_port={master_port}"
    )

    cleanup_gazebo()
    write_and_verify_yaml(vel_xy_p, vel_xy_d)

    if os.path.exists(CSV_PATH):
        os.remove(CSV_PATH)

    gazebo = None
    follower = None

    launch_cmd = make_ros_env_cmd(
        "ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py",
        master_port
    )

    try:
        gazebo = subprocess.Popen(launch_cmd, preexec_fn=os.setsid)
        print("[INFO] Gazebo/bringup arrancado", flush=True)

        time.sleep(3)

        follower_cmd = make_follower_cmd(master_port, run_id)
        follower = subprocess.Popen(follower_cmd)
        print("[INFO] trajectory_follower arrancado", flush=True)

        ok = send_takeoff(master_port, "/simple_drone/takeoff")
        print(f"[INFO] Takeoff enviado (ok={ok})", flush=True)

        time.sleep(RUN_SECONDS)

    finally:
        terminate_process_hard(follower, timeout=5)
        cleanup_followers()
        cleanup_gazebo()

    if not os.path.isfile(CSV_PATH):
        raise RuntimeError(
            f"No se generó {CSV_PATH}. "
            f"Busca dónde lo escribe tu follower con: find /ros2_ws -name trajectory_errors.csv"
        )

    df = pd.read_csv(CSV_PATH)
    if "err_norm" not in df.columns:
        raise RuntimeError(f"El CSV no tiene columna 'err_norm'. Columnas: {list(df.columns)}")

    cost = float((df["err_norm"] ** 2).mean())
    print(f"[RESULT] coste={cost:.6f}", flush=True)
    log_line(f"RUN_END_OK run_id={run_id} cost={cost}")
    return cost


def main():
    best_p = None
    best_d = None
    best_cost = float("inf")

    print("[INFO] Empezando FASE 2 (barrido velocityXY P,D)...", flush=True)

    run_id = 0
    for vel_xy_p, vel_xy_d in itertools.product(vel_xy_p_vals, vel_xy_d_vals):
        run_id += 1
        try:
            cost = run_sim_and_cost(vel_xy_p, vel_xy_d, run_id)
        except Exception as e:
            print("[RUN_BAD]", f"vel_xy_p={vel_xy_p} vel_xy_d={vel_xy_d} -> {repr(e)}", flush=True)
            log_line(f"RUN_END_BAD run_id={run_id} err={repr(e)}")
            continue

        if cost < best_cost:
            best_cost = cost
            best_p = vel_xy_p
            best_d = vel_xy_d
            print(f"[BEST] vel_xy_p={best_p} vel_xy_d={best_d} coste={best_cost:.6f}", flush=True)

    print("\n[FINAL] Mejores velocityXY (P,D):", (best_p, best_d), "con coste medio:", best_cost, flush=True)
    print("\n# Para copiar/pegar:", flush=True)
    print(f"BEST_VEL_XY_P = {best_p}", flush=True)
    print(f"BEST_VEL_XY_D = {best_d}", flush=True)


if __name__ == "__main__":
    main()
