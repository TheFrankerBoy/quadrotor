import subprocess
import yaml
import itertools
import time
import os
import signal
import pandas as pd
from datetime import datetime

YAML_PATH = "/ros2_ws/src/sjtu_drone/sjtu_drone_bringup/config/drone.yaml"

BASE_MASTER_PORT = 11345     # puerto base
PORT_SPAN = 50               # cuántos puertos “rotamos” (11345..11394)

CSV_PATH = "trajectory_errors.csv"
RUN_SECONDS = 90

# =========================
# FASES DE TUNING (prioridad)
# 1) roll/pitch (P,D) con vel_xy_p fijo
# 2) vel_xy (P,D) con roll/pitch fijado al mejor
# 3) (opcional) positionXY P con lo anterior fijado
# =========================

# Valores base del YAML (los que no vas a tocar al principio)
BASE = {
    "rollpitchProportionalGain": 10.0,
    "rollpitchDifferentialGain": 5.0,
    "velocityXYProportionalGain": 5.0,
    "velocityXYDifferentialGain": 2.3,
    "positionXYProportionalGain": 1.1,
}

# ---------- FASE 1: actitud (roll/pitch) ----------
# Mantén velocityXYProportionalGain fijo mientras estabilizas actitud
roll_p_vals = [6.0, 8.0, 10.0, 12.0, 14.0]
roll_d_vals = [2.0, 3.0, 4.0, 5.0, 6.0, 8.0]
vel_xy_p_vals = [BASE["velocityXYProportionalGain"]]  # fijo

# Si quieres: también barrer rollpitchLimit SOLO si ves saturación
# rollpitch_limit_vals = [0.4, 0.5, 0.6]  # opcional


# ---------- FASE 2: velocidad XY ----------
# Cuando encuentres el mejor (roll_p, roll_d) de Fase 1:
# fija esos dos en el YAML y barre velocityXYProportionalGain y velocityXYDifferentialGain
vel_xy_p_vals_phase2 = [3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
vel_xy_d_vals_phase2 = [0.5, 1.0, 1.5, 2.3, 3.0, 4.0]

# En esta fase NO barres roll_p/roll_d (los dejas en el mejor)
# roll_p_vals = [BEST_ROLL_P]
# roll_d_vals = [BEST_ROLL_D]


# ---------- FASE 3 (opcional): posición XY ----------
# Solo si ya va estable y quieres afinar captura/seguimiento en waypoints
pos_xy_p_vals_phase3 = [0.6, 0.9, 1.1, 1.4, 1.8, 2.2, 2.6]
# pos_xy_d_vals_phase3 = [0.0, 0.1, 0.2, 0.3]  # opcional (si activas D en posición)


LOG_PATH = "tuning_runs.log"
def log_line(msg: str):
    ts = datetime.now().isoformat(timespec="seconds")
    line = f"[{ts}] {msg}\n"
    with open(LOG_PATH, "a", encoding="utf-8") as f:
        f.write(line)
        f.flush()

def cleanup_followers():
    # Mata cualquier proceso que contenga el nombre del nodo o el ejecutable
    subprocess.run(
        ["bash", "-lc",
         "pkill -9 -f trajectory_follower || true; "
         "pkill -9 -f trajectory_follower_quintic_open_endzero || true"
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False
    )

    # Espera a que no quede ninguno (hasta 5s)
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
    """
    Espera a que exista el tópico de takeoff y tenga suscriptores, y publica varias veces.
    Devuelve True si aparentemente se pudo publicar con suscriptor presente.
    """
    # 1) Espera a que exista el tópico
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

    # 2) Espera a que haya al menos 1 suscriptor (si el CLI lo soporta)
    #    (Si falla el comando, seguimos igualmente)
    for _ in range(retries):
        cmd = f"ros2 topic info {topic} -v | grep -q 'Subscription count: [1-9]'"
        r = subprocess.run(
            make_ros_env_cmd(cmd, master_port),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        if r.returncode == 0:
            break
        time.sleep(wait_s)

    # 3) Publica varias veces (esto arregla carreras de arranque)
    for i in range(8):
        subprocess.run(
            make_ros_env_cmd(f"ros2 topic pub -1 {topic} std_msgs/msg/Empty '{{}}'", master_port),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
        )
        time.sleep(0.2)

    return True

def make_ros_env_cmd(cmd: str, master_port: int) -> list:
    """
    Ejecuta un comando con el entorno ROS y con GAZEBO_MASTER_URI específico.
    Ojo: el master_port debe ser el mismo para gzserver y gzclient (launch).
    """
    return [
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash && "
        "source /ros2_ws/install/setup.bash && "
        f"export GAZEBO_MASTER_URI=http://127.0.0.1:{master_port} && "
        f"{cmd}"
    ]


def cleanup_gazebo():
    """
    Mata Gazebo y espera a que NO quede ninguno vivo.
    No usa 'ss' (no está en tu contenedor).
    """
    subprocess.run(
        ["bash", "-lc",
         "pkill -9 -x gzserver || true; "
         "pkill -9 -x gzclient || true; "
         "pkill -9 -f gazebo || true; "
         "pkill -9 -f robot_state_publisher || true; "
         "rm -f /tmp/gazebo* /tmp/.gazebo* 2>/dev/null || true"
        ],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
    )

    # Espera activa: hasta 5 s a que desaparezcan procesos
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


def kill_group(proc, timeout=15):
    """
    Mata el grupo de procesos (padre + hijos). Imprescindible para Gazebo.
    """
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def terminate_process(proc, timeout=10):
    """
    Termina un proceso normal (p.ej. follower).
    """
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


def run_sim_and_cost(params, run_id: int):
    p, d, vxy = params
    master_port = BASE_MASTER_PORT + (run_id % PORT_SPAN)

    print(f"\n[TEST #{run_id}] roll_p={p}, roll_d={d}, vel_xy_p={vxy} | GAZEBO_MASTER_URI=:{master_port}", flush=True)
    log_line(f"RUN_START run_id={run_id} roll_p={p} roll_d={d} vel_xy_p={vxy} master_port={master_port}")

    # 0) Limpieza previa
    cleanup_gazebo()

    # 1) Editar YAML
    with open(YAML_PATH, "r") as f:
        data = yaml.safe_load(f)

    data["rollpitchProportionalGain"] = p
    data["rollpitchDifferentialGain"] = d
    data["velocityXYProportionalGain"] = vxy

    with open(YAML_PATH, "w") as f:
        yaml.safe_dump(data, f)

    # 2) Borrar CSV anterior
    if os.path.exists(CSV_PATH):
        os.remove(CSV_PATH)

    gazebo = None
    follower = None

    # 3) Comandos (con el master_port elegido)
    launch_cmd = make_ros_env_cmd(
        "ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py",
        master_port
    )

    takeoff_cmd = make_ros_env_cmd(
        "ros2 topic pub -1 /simple_drone/takeoff std_msgs/msg/Empty {}",
        master_port
    )

    follower_cmd = make_ros_env_cmd(
        "ros2 run sjtu_drone_control trajectory_follower",
        master_port
    )

    try:
        # 4) Arrancar Gazebo/bringup en nuevo grupo
        gazebo = subprocess.Popen(launch_cmd, preexec_fn=os.setsid)
        print("[INFO] Gazebo/bringup arrancado", flush=True)

        time.sleep(3)

        # 6) Arrancar follower
        follower_cmd = make_follower_cmd(master_port, run_id)
        follower = subprocess.Popen(follower_cmd)
        print("[INFO] trajectory_follower arrancado", flush=True)



        # 5) Takeoff
        ok = send_takeoff(master_port, "/simple_drone/takeoff")
        print(f"[INFO] Takeoff enviado (ok={ok})", flush=True)
        time.sleep(RUN_SECONDS)


        

    finally:
        terminate_process_hard(follower, timeout=5)
        cleanup_followers()

        cleanup_gazebo()

    # 7) Leer CSV
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


best_params = None
best_cost = float("inf")

print("[INFO] Empezando barrido...", flush=True)

run_id = 0
for params in itertools.product(roll_p_vals, roll_d_vals, vel_xy_p_vals):
    run_id += 1
    try:
        cost = run_sim_and_cost(params, run_id)
    except Exception as e:
        print("[ERROR]", repr(e), flush=True)
        continue

    if cost < best_cost:
        best_cost = cost
        best_params = params
        print(f"[BEST] params={best_params} coste={best_cost:.6f}", flush=True)

print("\n[FINAL] Mejores parámetros:", best_params, "con coste medio:", best_cost, flush=True)

