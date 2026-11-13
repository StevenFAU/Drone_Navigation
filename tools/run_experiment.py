#!/usr/bin/env python3
"""Run the full mission N times in fresh headless sims, then analyse the results.

Each run: start the sim (headless, GPS/baro noise unless --no-noise) -> set a random wind
(speed uniform in [--wind-min, --wind-max] m/s, random direction; gusts come from the world's
wind plugin) -> `ros2 run drone_mission mission` -> stop the sim. A fresh sim per run means every
run starts from the same spot with a clean EKF.

    python3 tools/run_experiment.py                      # 10 runs -> results/<time>/
    python3 tools/run_experiment.py --runs 3 --wind-max 0 --no-noise
    python3 tools/run_experiment.py --runs 5 --wind-max 0 --gps-denied   # indoor, optical flow
    python3 tools/analyze_experiment.py results/<time>   # (re)make plots + summary

Needs nothing else running: it refuses to start if a sim is already up. ~2.5 min per run.
"""
import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def sh(cmd, **kw):
    """Run a command with the project's ROS/Gazebo environment."""
    return subprocess.run(['bash', '-c', f'source "{ROOT}/setup/env.sh" && {cmd}'], **kw)


def sim_processes():
    out = subprocess.run(['ps', '-eo', 'pid,cmd'], capture_output=True, text=True).stdout
    keys = ('build/sitl/bin/arducopter', 'gz-sim-main', 'mavros_node', 'parameter_bridge /camera')
    me = str(os.getpid())
    return [ln.strip() for ln in out.splitlines()
            if any(k in ln for k in keys) and not ln.strip().startswith(me)]


def start_sim(out_path, noise, gps_denied=False):
    args = ([f'{ROOT}/scripts/start_sim.sh', '--headless'] + (['--noise'] if noise else [])
            + (['--gps-denied'] if gps_denied else []))
    f = open(out_path, 'w')
    proc = subprocess.Popen(args, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT,
                            start_new_session=True)
    deadline = time.time() + 240
    while time.time() < deadline:
        time.sleep(2)
        text = open(out_path).read()
        if 'READY' in text:
            return proc
        if proc.poll() is not None or 'still running' in text:
            break
    stop_sim(proc)
    raise RuntimeError(f'sim did not become ready (see {out_path})')


def stop_sim(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGTERM)    # start_sim.sh's trap stops all its programs
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
    for _ in range(10):
        if not sim_processes():
            return
        time.sleep(1)
    raise RuntimeError('sim processes still running after stop:\n' + '\n'.join(sim_processes()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--runs', type=int, default=10)
    ap.add_argument('--wind-min', type=float, default=1.0, help='m/s')
    ap.add_argument('--wind-max', type=float, default=4.0, help='m/s (0 = no wind)')
    ap.add_argument('--no-noise', action='store_true', help='no GPS/baro noise')
    ap.add_argument('--gps-denied', action='store_true', help='optical flow + rangefinder, no GPS')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--params-file', default='', help='mission params YAML (default: built-in route)')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    if sim_processes():
        sys.exit('A sim is already running; stop it first:\n' + '\n'.join(sim_processes()))
    out = args.out or os.path.join(ROOT, 'results', datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)
    rng = random.Random(args.seed)
    config = {'runs': args.runs, 'wind_min': args.wind_min, 'wind_max': args.wind_max,
              'noise': not args.no_noise, 'gps_denied': args.gps_denied, 'seed': args.seed, 'params_file': args.params_file,
              'started': datetime.now().isoformat(timespec='seconds'), 'run_conditions': []}
    print(f'experiment -> {out}')

    for i in range(1, args.runs + 1):
        run_dir = os.path.join(out, f'run_{i:02d}')
        os.makedirs(run_dir, exist_ok=True)
        speed = rng.uniform(args.wind_min, args.wind_max) if args.wind_max > 0 else 0.0
        direction = rng.uniform(-math.pi, math.pi)
        wx, wy = speed * math.cos(direction), speed * math.sin(direction)
        cond = {'run': i, 'wind_speed': round(speed, 2),
                'wind_dir_deg': round(math.degrees(direction), 1),   # ENU: direction it blows TO
                'wind_x': round(wx, 3), 'wind_y': round(wy, 3)}
        config['run_conditions'].append(cond)
        print(f'[run {i}/{args.runs}] wind {speed:.1f} m/s toward {math.degrees(direction):.0f} deg; starting sim',
              flush=True)
        t0 = time.time()
        proc = start_sim(os.path.join(run_dir, 'sim.out'), not args.no_noise, args.gps_denied)
        try:
            sh(f"gz topic -t /world/aruco_world/wind -m gz.msgs.Wind "
               f"-p 'linear_velocity: {{x: {wx:.3f}, y: {wy:.3f}}}, enable_wind: true'", check=True)
            ros_args = f'-p timeout_scale:=1.0 -p log_dir:={run_dir}'
            if args.params_file:
                ros_args += f' --params-file {os.path.abspath(args.params_file)} -p log_dir:={run_dir}'
            with open(os.path.join(run_dir, 'mission.out'), 'w') as f:
                res = sh(f'ros2 run drone_mission mission --ros-args {ros_args}',
                         stdout=f, stderr=subprocess.STDOUT, timeout=600)
            cond['exit_code'] = res.returncode
        except subprocess.TimeoutExpired:
            cond['exit_code'] = 'timeout'
        finally:
            stop_sim(proc)
        cond['wall_s'] = round(time.time() - t0, 1)
        tail = open(os.path.join(run_dir, 'mission.out')).read().strip().splitlines()[-1:]
        print(f'           exit {cond["exit_code"]} after {cond["wall_s"]:.0f} s: {tail[0][-110:] if tail else ""}',
              flush=True)
        with open(os.path.join(out, 'experiment.json'), 'w') as f:
            json.dump(config, f, indent=2)

    print('analysing...')
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'analyze_experiment.py'), out])


if __name__ == '__main__':
    main()
