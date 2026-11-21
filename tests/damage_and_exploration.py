"""Run damage-response, scouting, and rotation regressions with KIT_DIR and IMAGE."""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import runpy
from tempfile import TemporaryDirectory

from firing_replay import ask, docker_bot, load_recording, start_bot, wall, write_arena


def scenarios(sample):
    drone = {"kind": "npc", "id": 104, "pos": [35, 0, 0], "vel": [0, 0, 0],
             "aim": [math.pi, 0], "hp": 350, "max_hp": 350}

    def frame(t, hp, *, actors=(), events=()):
        o = deepcopy(sample)
        for key in ("projectiles", "crates", "cubes", "pickups", "sounds", "beams", "drops"):
            o[key] = []
        o.update(t=t, tick=round(t * 20), alive=16, mechs=list(actors), events=list(events), offer=None)
        o["self"].update(pos=[0, 0, 0], vel=[0, 0, 0], aim=[0, 0], hp=hp,
                         max_hp=1400, cubes=0, grounded=True, in_zone=True, energy=100)
        o["zone"].update(center=[0, 0], next_center=[0, 0], radius=200, next_radius=200,
                         shrink_start=200, shrink_end=260, dps=20)
        return o

    def hit(weapon, by):
        return {"type": "damaged", "weapon": weapon, "by": by, "amount": 45, "bearing": 0}

    def moving(a):
        return math.hypot(*a["move"]) > 0.1 or a["jump"]

    def sustained_escape(actions, traces):
        # Check actual leg commands after each hit; a changed label alone cannot pass.
        return all(moving(actions[i]) for i in (1, 2))

    # Walk around the end of the wall, take a hit, then get back behind it
    # while the drone remains in the official five-second visual memory.
    in_cover = [frame(5, 420, actors=[drone]), frame(10, 375, actors=[drone], events=[hit("npc", 104)]),
                frame(11.8, 375, actors=[drone])]
    in_cover[1]["self"]["pos"][1] = 30
    in_cover[2]["self"]["pos"][1] = 10
    explore = [frame(i / 10, 1400) for i in range(121)]
    explore[-1]["pickups"] = [{"id": 8001, "kind": "lightning", "pos": [0, 0, 0]}]

    def explores_then_collects(actions, traces):
        points = [o["self"]["pos"] for o in explore]
        traveled = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        last_pickup = explore[-1]["pickups"][0]["pos"]
        moving_count = sum(moving(a) for a in actions[:-1])
        print(f"Flat-ground fixture: {moving_count}/120 scouting actions moved, {traveled:.1f} m traveled")
        return (moving_count >= 80 and traveled > 30 and
                traces[-1]["mode"] == "SNOWBALL" and math.dist(traces[-1]["destination"], last_pickup) < 1)

    rotation = frame(60, 1150)
    rotation["self"].update(energy=0, in_zone=False)
    rotation["zone"].update(center=[20, 0], next_center=[20, 0], radius=13, next_radius=13,
                            shrink_start=100, shrink_end=130, dps=160)
    enclosure = [wall(6, -5.5, 7, 5.5, 30), wall(15, -5.5, 16, 5.5, 30),
                 wall(7, -5.5, 15, -4.5, 30), wall(7, 4.5, 15, 5.5, 30)]

    def alternate_destination(actions, traces):
        x, y, _ = traces[0]["destination"]
        in_enclosure = 6 <= x <= 16 and -5.5 <= y <= 5.5
        return moving(actions[0]) and math.dist((x, y), (20, 0)) < 13 and not in_enclosure

    return [
        ("low health and repeated exposed drone hits produce escape movement", [],
         [frame(10, 540, actors=[drone]),
          frame(10.1, 495, actors=[drone], events=[hit("npc", 104)]),
          frame(10.8, 450, actors=[drone], events=[hit("npc", 104)])], sustained_escape, None),
        ("70-percent-health mech moves after a hit from an unseen bearing", [],
         [frame(10, 1025), frame(10.1, 980, events=[hit("gun", 5)]), frame(16.1, 980)],
         lambda actions, traces: moving(actions[1]) and not any(actions[1]["fire"]) and
             traces[-1]["mode"] == "REGEN" and not moving(actions[-1]), None),
        ("solid cover and a passive drone allow uninterrupted healing", [wall(14, -12, 18, 12, 18)],
         in_cover, lambda actions, traces: all(traces[i]["mode"] == "REGEN" and not moving(actions[i]) and
             not any(actions[i]["fire"]) for i in (0, 2)), None),
        ("healthy spawn explores empty ground for 120 decisions, then prioritizes visible loot", [],
         explore, explores_then_collects, advance_on_flat_ground),
        ("sealed courtyard is skipped when an alternative in the safe zone is reachable",
         enclosure, [rotation], alternate_destination, None),
    ]


def advance_on_flat_ground(previous, action, obs):
    """Advance only planar movement across two physics ticks on the empty map."""
    stats = previous["self"]["stats"]
    speed = stats["boost_speed"] if action["boost"] else stats["walk_speed"]
    desired = [speed * direction for direction in action["move"]]
    velocity = previous["self"]["vel"][:2]
    position = previous["self"]["pos"][:2]
    for _ in range(2):
        delta = [desired[i] - velocity[i] for i in range(2)]
        length = math.hypot(*delta)
        fraction = 1 if length == 0 else min(1, 0.05 * stats["ground_accel"] / length)
        for i in range(2):
            velocity[i] += delta[i] * fraction
            position[i] += velocity[i] * 0.05
    obs["self"].update(pos=[*position, 0], vel=[*velocity, 0])
    if obs["pickups"]:
        obs["pickups"][0]["pos"] = [position[0], position[1] + 8, 0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kit", type=Path)
    parser.add_argument("image")
    args = parser.parse_args()
    hello, observations, end = load_recording(args.kit)
    schema = runpy.run_path(str(args.kit / "check/check.py"))["Schema"](args.kit / "mechfight.schema.json")
    cases = scenarios(observations[0])
    failed = 0
    for name, prisms, frames, expected, advance in cases:
        with TemporaryDirectory() as folder:
            arena = Path(folder)
            write_arena(arena, prisms)
            diagnostics, actions = [], []
            with docker_bot(args.image, arena, diagnostics, trace=True) as (send, receive):
                start_bot(send, receive, hello)
                for i, obs in enumerate(frames):
                    if advance and i:
                        advance(frames[i - 1], actions[-1], obs)
                    assert not schema.errors(obs, "Observation"), name
                    action = ask(send, receive, obs)
                    assert not schema.errors(action, "Action"), (name, action)
                    actions.append(action)
                send(end)
            traces = [json.loads(line) for line in diagnostics]
            assert [t["tick"] for t in traces] == [o["tick"] for o in frames], name
            if expected(actions, traces):
                print("PASS", name)
            else:
                failed += 1
                samples = list(zip(actions, traces))
                if len(samples) > 8:
                    samples = samples[:2] + samples[-2:]
                print("FAIL", name, [(t["mode"], t["movement_reason"], a["move"], a["jump"])
                                     for a, t in samples])
    print(f"{len(cases) - failed}/{len(cases)} damage, scouting, and rotation cases passed")
    return bool(failed)


if __name__ == "__main__":
    raise SystemExit(main())
