"""Run focused gun checks and replay the official kit stream against Docker images.

Usage: python tests/firing_replay.py KIT_DIR UPDATED_IMAGE [BASELINE_IMAGE ...]
Stage images with the repository's Dockerfile before running this script.
"""

import argparse
from collections import Counter
from contextlib import contextmanager
from copy import deepcopy
import gzip
import json
import math
from pathlib import Path
from queue import Queue
import subprocess
from tempfile import TemporaryDirectory, TemporaryFile
from threading import Thread


DOCKER_LIMITS = [
    "--network", "none", "--cpus", "1", "--memory", "512m",
    "--memory-swap", "512m", "--pids-limit", "128", "--cap-drop", "ALL",
    "--read-only", "--tmpfs", "/tmp:rw,size=64m", "--user", "65534:65534",
]


@contextmanager
def docker_bot(image, arena, diagnostics=None):
    command = ["docker", "run", "--rm", "-i", *DOCKER_LIMITS,
               "--mount", f"type=bind,source={arena.resolve()},target=/arena,readonly", image]
    with TemporaryFile(mode="w+", encoding="utf-8") as errors:
        with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=errors, text=True, encoding="utf-8") as bot:
            responses = Queue()

            def read_output():
                for line in bot.stdout:
                    responses.put(line)
                responses.put(None)

            Thread(target=read_output, daemon=True).start()

            def send(message):
                bot.stdin.write(json.dumps(message, separators=(",", ":")) + "\n")
                bot.stdin.flush()

            def receive():
                line = responses.get(timeout=10)
                if line is None:
                    errors.seek(0)
                    raise AssertionError(f"{image} closed stdout: {errors.read()}")
                return json.loads(line)

            try:
                yield send, receive
            finally:
                bot.stdin.close()
                try:
                    bot.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    bot.kill()
                    raise
                errors.seek(0)
                assert bot.returncode == 0, (image, bot.returncode, errors.read())
                for line in errors:
                    assert diagnostics is not None, (image, line)
                    diagnostics.append(line.rstrip("\n"))


def load_recording(kit):
    with gzip.open(kit / "check/stream.jsonl.gz", "rt", encoding="utf-8") as stream:
        hello, *observations, end = map(json.loads, stream)
    hello["map"]["dir"] = "/arena"
    return hello, observations, end


def start_bot(send, receive, hello):
    send(hello)
    assert receive()["type"] == "ready"


def ask(send, receive, obs):
    send(obs)
    action = receive()
    assert action["tick"] == obs["tick"], action
    return action


def replay(image, arena, hello, observations, end, diagnostics=None):
    with docker_bot(image, arena, diagnostics) as (send, receive):
        start_bot(send, receive, hello)
        actions = [ask(send, receive, obs) for obs in observations]
        send(end)
    return actions


def write_arena(arena, prisms):
    version = arena / "v2"
    version.mkdir()
    world = {
        "bounds": {"minX": -400, "minY": -400, "maxX": 400, "maxY": 400},
        "terrain": {"minX": -400, "minY": -400, "step": 800, "scale": 0.01,
                    "nx": 2, "ny": 2, "file": "terrain.i16"},
        "prisms": prisms,
    }
    (version / "world.json").write_text(json.dumps(world), encoding="utf-8")
    (version / "terrain.i16").write_bytes(bytes(8))


def wall(x1, y1, x2, y2, top):
    return {"base": 0, "top": top, "ring": [x1, y1, x2, y1, x2, y2, x1, y2]}


def enemy(actor_id, x, y, facing=False):
    return {"id": actor_id, "kind": "mech", "pos": [x, y, 0], "vel": [0, 0, 0], "cubes": 0,
            "aim": [math.pi, math.atan2(-2.5, x)] if facing else [0, 0],
            "weapons": ["gun"], "charging": False, "hp": 1400}


def aimed_at(actor):
    x, y, _ = actor["pos"]
    return [math.atan2(y, x), math.atan2(-2.5, math.hypot(x, y))]


def focused_cases(image, hello, sample, end):
    def scenario(actors, aim):
        obs = deepcopy(sample)
        obs.update(t=10, tick=200, mechs=actors, offer=None)
        for field in ("beams", "crates", "cubes", "drops", "events", "pickups", "projectiles", "sounds"):
            obs[field] = []
        obs["self"].update(pos=[0, 0, 0], vel=[0, 0, 0], aim=aim, grounded=True, in_zone=True)
        obs["zone"].update(center=[0, 0], radius=200, next_center=[0, 0], next_radius=200)
        return obs

    def verify(send, receive, name, obs, fire, aim=None, state=None, stationary=False):
        start_bot(send, receive, hello)
        action = ask(send, receive, obs)
        assert action["fire"] == [fire, False], (name, action)
        if aim is not None:
            assert math.dist(action["aim"], aim) < 1e-6, (name, action)
        if state is not None:
            assert action["debug"]["label"] == state, (name, action)
        if stationary:
            assert action["move"] == [0, 0], (name, action)
        print(f"PASS {name}")
        return action

    target = enemy(4, 40, 0)
    face_to_face = enemy(4, 40, 0, facing=True)
    other = enemy(5, 0, 110)
    aim = aimed_at(target)

    with TemporaryDirectory() as directory:
        arena = Path(directory)
        write_arena(arena, [])
        with docker_bot(image, arena) as (send, receive):
            obs = scenario([target], aim)
            action = verify(send, receive, "visible opponent facing away, no attack evidence", obs, True, aim)
            assert action["move"][0] < -0.15, action

            stuck = scenario([face_to_face], aim)
            stuck["zone"].update(radius=3, next_radius=3)
            verify(send, receive, "stationary with a clear shot", stuck, True, aim, "rotate", stationary=True)

            recovering = scenario([face_to_face], aim)
            recovering["self"]["in_zone"] = False
            recovering["zone"].update(center=[6, 0], radius=3, next_center=[6, 0], next_radius=3)
            action = verify(send, receive, "moving toward a clear target during zone recovery",
                            recovering, True, aim, "recover-zone")
            assert action["move"][0] > 0.05, action

            verify(send, receive, "aims without firing while the turret is misaligned",
                   scenario([target], [0.3, aim[1]]), False, aim)
            verify(send, receive, "takes a farther aligned shot before aiming at a closer target",
                   scenario([target, other], aimed_at(other)), True, aimed_at(other))

            for field, value in [("ammo", 0), ("cooldown", 0.2), ("reload", 1)]:
                unavailable = scenario([target], aim)
                unavailable["self"]["weapons"][0][field] = value
                verify(send, receive, f"gun unavailable ({field})", unavailable, False)

            distant = enemy(4, 240, 0)
            verify(send, receive, "out of gun range", scenario([distant], aimed_at(distant)), False)
            send(end)

    with TemporaryDirectory() as directory:
        arena = Path(directory)
        # The 8 m eye ray clears this low wall while the shot to the torso hits it.
        write_arena(arena, [wall(17, -6, 20, 6, 6.35), wall(0.45, 48, 0.85, 53, 6.35)])
        with docker_bot(image, arena) as (send, receive):
            verify(send, receive, "wall blocks the shot during withdrawal",
                   scenario([target], aim), False, state="withdraw")
            verify(send, receive, "selects the clear opponent when the nearer one is obstructed",
                   scenario([target, other], aimed_at(other)), True, aimed_at(other))
            yaw, pitch = aimed_at(other)
            verify(send, receive, "obstruction on the actual firing line blocks the trigger",
                   scenario([other], [yaw - 0.012, pitch]), False, aimed_at(other))
            send(end)

    with TemporaryDirectory() as directory:
        arena = Path(directory)
        write_arena(arena, [wall(17, -6, 20, 6, 7.8)])
        with docker_bot(image, arena) as (send, receive):
            verify(send, receive, "shelter suppresses a clear shot at a different opponent",
                   scenario([target, other], aimed_at(other)), False, state="hold-wall", stationary=True)
            verify(send, receive, "same shot is clear without the threat requiring shelter",
                   scenario([other], aimed_at(other)), True, aimed_at(other))
            send(end)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("kit", type=Path)
    cli.add_argument("images", nargs="+")
    cli.add_argument("--report", type=Path, help="save per-tick actions for comparison")
    args = cli.parse_args()
    hello, observations, end = load_recording(args.kit)
    focused_cases(args.images[0], hello, observations[0], end)
    visible = [any(m["kind"] == "mech" for m in obs["mechs"]) for obs in observations]
    print(f"{sum(visible)} of {len(visible)} recorded observations contain a visible opponent")
    runs = {}
    for image in args.images:
        actions = replay(image, args.kit / "arena", hello, observations, end)
        fired = [any(action["fire"]) for action in actions]
        assert all(not action["fire"][1] for action in actions), image
        print(f"{image}: {sum(fired)} trigger ticks, {sum(a and b for a, b in zip(fired, visible))} with an opponent visible")
        runs[image] = actions

    updated = runs[args.images[0]]
    for image in args.images[1:]:
        fields = Counter(field for a, b in zip(updated, runs[image])
                         for field in ("move", "boost", "jump", "reload", "pick") if a.get(field) != b.get(field))
        print(f"Updated vs {image}, changed command fields: {dict(fields)}")
    if args.report:
        args.report.write_text(json.dumps({"opponent_visible": visible, "actions": runs}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
