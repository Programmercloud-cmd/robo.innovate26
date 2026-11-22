"""Synthetic-only network/dropout/reconnect test on a separate Isaac process."""
import argparse
import json
import socket
import subprocess
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--root", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
run = args.output/"run"
log = (args.output/"process.log").open("w")
process = subprocess.Popen(["bash", str(args.root/"isaac/launch.sh"), "--output", str(run), "--port", "8767"], stdout=log, stderr=subprocess.STDOUT)


def wait_for(predicate, timeout=20):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        if process.poll() is not None:
            raise RuntimeError("owned test simulator exited unexpectedly")
        value = predicate()
        if value:
            return value
        time.sleep(.03)
    raise TimeoutError("test condition did not arrive")


def state():
    try:
        return json.loads((run/"latest.json").read_text())
    except (FileNotFoundError, ValueError):
        return {}


def connect():
    try:
        return socket.create_connection(("127.0.0.1", 8767), timeout=.5)
    except OSError:
        return None


try:
    client = wait_for(connect)
    q = [.4, .8, -.3, -1.6, .35, 2.3, .7]
    frame = {"schema": "fr3.telemetry.v1", "seq": 0, "unix_ns": time.time_ns(),
             "q": q, "dq": [0.]*7, "source": "synthetic_socket_smoke_not_hardware"}
    client.sendall((json.dumps(frame)+"\n").encode())
    fresh = wait_for(lambda: (v if (v:=state()).get("seq")==0 else None))
    client.sendall(b'{"schema":"bad.schema"}\n')
    client.close()
    stale = wait_for(lambda: (v if (v:=state()).get("stale") else None))
    if stale["q_isaac"] != fresh["q_isaac"]:
        raise AssertionError("dropout changed simulated joint position")
    client = wait_for(connect)
    frame["seq"] = 1
    frame["q"][0] += .05
    client.sendall((json.dumps(frame)+"\n").encode())
    resumed = wait_for(lambda: (v if (v:=state()).get("seq")==1 and not v.get("stale") else None))
    client.close()
    if resumed["q_isaac"] == stale["q_isaac"]:
        raise AssertionError("new valid sample was not applied")
    (args.output/"evidence.json").write_text(json.dumps({"fresh": fresh, "stale": stale, "resumed": resumed,
        "hardware_commands": 0, "test_source": "synthetic socket frames only; separate PhysX process on8767"}, indent=2))
    print("PASS: synthetic telemetry, invalid-frame rejection, stale hold, reconnection and resumed joint updates")
finally:
    process.terminate()
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
    log.close()
