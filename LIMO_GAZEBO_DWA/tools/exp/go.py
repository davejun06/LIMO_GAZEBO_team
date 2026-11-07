#!/usr/bin/env python3
"""Start signal in one command: puts the car behind the start line, checks, starts, measures, stops on a safety rule.
Ctrl-C or any exit closes the gate with a limited number of stop messages and reads speed back.
usage (windows 1 to 5 up): python3 tools/exp/go.py [laps] [loc_abort]    Default 1 lap, 0.25. Watching only, results in logs/exp/my-run."""
import json, os, re, signal, subprocess, sys, threading, time
OUT = 'logs/exp/my-run'; CSV = 'src/Raceline-Optimization/outputs/track_map3_line_pp.csv'
ROS = 'source ' + os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'env.sh') + '; '
laps = sys.argv[1] if len(sys.argv) > 1 else '1'
loc_abort = sys.argv[2] if len(sys.argv) > 2 else '0.25'
started = False; mon = None
os.makedirs(OUT, exist_ok=True)
def sh(c, t):
    try:
        return subprocess.run(['bash', '-c', ROS + c], capture_output=True, text=True, timeout=t).stdout
    except subprocess.TimeoutExpired:
        return ''
def finish():
    if not started:
        return
    for n in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(n, signal.SIG_IGN)
    sh('timeout 6 ros2 topic pub --times 6 --rate 5 /control_selector std_msgs/msg/String "{data: stop}"', 10)
    time.sleep(1)
    c = re.search(r'speed: (\S+)', sh('timeout 4 ros2 topic echo /final_ackermann_cmd --once', 6))
    v = re.search(r'linear:\s+x: (\S+)', sh('timeout 4 ros2 topic echo /odom --once', 6))
    moving = not v or abs(float(v.group(1))) > 0.02 or not c or abs(float(c.group(1))) > 0.001
    msg = f"gate closed; final command {c.group(1) if c else '?'} odometry {v.group(1) if v else '?'}"
    if moving:
        msg += '\nWARNING: the car may still be moving. Stop the planner, Pure Pursuit and converter windows now (Ctrl-C).'
    try:                                 # the terminal may be gone
        print(msg, flush=True)
    except OSError:
        pass
def bye(*_):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, bye)
signal.signal(signal.SIGHUP, bye)
code = 1
try:
    world = sh('timeout 8 gz model --list 2>/dev/null; for o in 1 2 3; do timeout 8 gz model -m map1_obstacle_$o -p 2>/dev/null | head -1; done', 40)
    if len(re.findall(r'^-?\d+\.?\d* ', world, re.M)) < 3:
        sys.exit('ABORT: obstacles map1_obstacle_1..3 not in this world, no start signal sent')
    if subprocess.run(['python3', 'tools/exp/place_start.py'], stderr=subprocess.DEVNULL).returncode:
        sys.exit('ABORT: placement failed; are Gazebo and localization up?')
    time.sleep(3)
    for attempt in range(3):
        out = sh('python3 tools/exp/preflight.py alpha1=0.2 alpha2=0.2 alpha3=0.2 alpha4=0.2 update_min_d=0.1 update_min_a=0.1 2>/dev/null', 150)
        open(OUT + '/preflight.txt', 'w').write(out)
        m = re.search(r'\{"fails": (\[.*\])\}', out)
        fails = [f for f in json.loads(m.group(1)) if f != 'no gzclient during runs (CPU)'] if m else ['preflight gave no result']
        if not fails:
            break
        print('preflight:', fails, flush=True)
        if not any(f.startswith('parameter queries') for f in fails):
            sys.exit('ABORT: no start signal sent')
        time.sleep(10)
    else:
        sys.exit('ABORT: slow parameter queries, no start signal sent')
    if subprocess.run(['python3', 'tools/exp/pp_ready.py'], stdout=subprocess.DEVNULL).returncode:
        sys.exit('ABORT: Pure Pursuit not ready, no start signal sent; are windows 3, 4, 5 up?')
    started = True
    with open(OUT + '/monitor.err', 'w') as e:
        mon = subprocess.Popen(['python3', 'tools/exp/run_monitor.py', '--exp', 'my-run', '--path', CSV, '--out', OUT + '/run.csv', '--laps', laps, '--timeout', str(300 * int(laps)), '--loc-abort', loc_abort], stderr=subprocess.PIPE, stdout=subprocess.DEVNULL, text=True)
        def no_start():
            if not signalled.is_set():
                print('ABORT: no start signal in 60 s: no contact stream, so no collision detection; the car was not started. Restart Gazebo.', flush=True)
                mon.send_signal(signal.SIGINT)
        signalled = threading.Event(); threading.Timer(60.0, no_start).start()
        for l in mon.stderr:
            e.write(l); e.flush()
            if 'start signal' in l:
                signalled.set()
            if re.search(r'STOP|lap \d|start signal|motion', l):
                print(l.rstrip()[:120], flush=True)
    mon.wait(); code = mon.returncode or 0
except KeyboardInterrupt:
    if mon and mon.poll() is None:
        mon.send_signal(signal.SIGINT)
        try:
            mon.wait(5)
        except subprocess.TimeoutExpired:
            mon.kill()
    code = 130
finally:
    finish()
    subprocess.run(['pkill', '-f', 'gz topic -u -e /gazebo/ifac_roboracer/physics/contacts'])   # the monitor's reader, if left behind
sys.exit(code)
