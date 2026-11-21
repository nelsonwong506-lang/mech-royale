"""Exercise vNext through its real JSON protocol. Run with KIT_DIR and a Docker image.

The 50 scenarios use small synthetic maps and the official kit's configuration.
Each case starts a fresh match with hello. Repeated observations within a case
preserve memory. The optional comparison uses the untouched 300-observation
recording, so its counts describe proposed actions, not simulated outcomes.
"""

import argparse
from collections import Counter, defaultdict
from collections.abc import Callable
from copy import deepcopy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import runpy
from tempfile import TemporaryDirectory

from firing_replay import ask, docker_bot, enemy, load_recording, replay, start_bot, wall, write_arena


MODES = {"COMBAT", "REGEN", "SNOWBALL", "ROTATE"}


@dataclass
class Frame:
    observation: dict
    action: dict
    trace: dict


@dataclass
class Case:
    group: str
    name: str
    map_name: str
    observations: list
    check: Callable[[list[Frame]], bool]


def mode(frame):
    return frame.trace["mode"]


def fired(frame, slot=0):
    return frame.action["fire"][slot]


def yaw_error(frame, target):
    return abs(math.remainder(frame.action["aim"][0] - target, 2 * math.pi))


def fixtures(hello, original):
    def mech(x=40, y=0, z=0, hp=1400, actor_id=4):
        result = enemy(actor_id, x, y)
        result.update(pos=[x, y, z], hp=hp)
        return result

    def weapon(kind, **updates):
        stats = deepcopy(hello["config"]["weapons"][kind])
        result = {"kind": kind, "stats": stats, "cooldown": 0}
        if kind == "laser":
            result["charge"] = 0
        return dict(result, **updates)

    def obs(*, actors=(), self_fields=None, outside=False, zone_fields=None, **fields):
        result = deepcopy(original)
        for key in ("beams", "crates", "cubes", "drops", "events", "pickups", "projectiles", "sounds"):
            result[key] = []
        result.update(t=10, tick=200, alive=16, offer=None, mechs=list(actors))
        result["self"].update(pos=[0, 0, 0], vel=[0, 0, 0], aim=[0, math.atan2(-2.5, 40)],
                              hp=1400, max_hp=1400, grounded=True, energy=100, cubes=0,
                              in_zone=not outside)
        result["self"].update(self_fields or {})
        result["zone"].update(center=[0, 0], radius=200, next_center=[0, 0], next_radius=200,
                              shrink_start=200, shrink_end=260, dps=20)
        if outside:
            result["zone"].update(center=[12, 0], radius=8, next_center=[12, 0], next_radius=8)
        result["zone"].update(zone_fields or {})
        result.update(fields)
        return result

    def gun(**changes):
        return dict(deepcopy(original["self"]["weapons"][0]), **changes)

    def arm(secondary, **overrides):
        return dict(weapons=[gun(ammo=0, reload=2), weapon(secondary)], **overrides)

    def pickup(kind="lightning", x=2, z=0):
        return [{"id": 8001, "kind": kind, "pos": [x, 0, z]}]

    def later(item, seconds=0.1):
        copy = deepcopy(item)
        copy.update(t=item["t"] + seconds, tick=item["tick"] + round(seconds * 20))
        return copy

    cases = []

    def case(group, name, observations, check, arena="open"):
        sequence = observations if isinstance(observations, list) else [observations]
        cases.append(Case(group, name, arena, sequence, check))

    def last(predicate):
        return lambda frames: predicate(frames[-1])

    def is_mode(expected):
        return last(lambda f: mode(f) == expected)

    def roof(position, grounded=True, **extra):
        return obs(self_fields=dict(pos=position, grounded=grounded, **extra))

    group = "protocol and mode arbitration"
    case(group, "fresh spawn seeks visible resources", obs(pickups=pickup(x=20)), is_mode("SNOWBALL"))
    case(group, "visible nearby rival activates combat", obs(actors=[mech()]), is_mode("COMBAT"))
    case(group, "combat owns movement when a distant pickup competes",
         obs(actors=[mech()], pickups=pickup(x=30)), is_mode("COMBAT"))
    case(group, "low HP with no contacts activates regeneration",
         obs(self_fields={"hp": 350}), is_mode("REGEN"))
    case(group, "late damaging storm owns movement during a duel",
         obs(actors=[mech(hp=1300)], outside=True, zone_fields={"dps": 160}),
         last(lambda f: mode(f) == "ROTATE" and f.action["move"][0] > 0))
    case(group, "tiny final circle permits stationary independent firing",
         obs(actors=[mech()], zone_fields={"radius": 3, "next_radius": 3}),
         last(lambda f: math.hypot(*f.action["move"]) < 0.1 and fired(f)))
    case(group, "action tracks a nonconsecutive observation tick",
         [obs(), obs(t=13.4, tick=268, actors=[mech()])],
         lambda fs: [f.trace["tick"] for f in fs] == [200, 268] and mode(fs[-1]) == "COMBAT")
    case(group, "visible target retains aim despite a conflicting sound",
         obs(actors=[mech()], sounds=[{"kind": "jet", "bearing": 2.5, "dist": "near", "age": 0}]),
         last(lambda f: yaw_error(f, 0) < 0.1))
    case(group, "empty scene does not trigger weapons", obs(), last(lambda f: not any(f.action["fire"])))
    case(group, "safe supply-drop capture holds position",
         obs(drops=[{"id": 20001, "pos": [1, 0, 0], "radius": 8, "capture_s": 4,
                     "lands_in": 0, "holder": 3, "progress": 1}]),
         last(lambda f: mode(f) == "SNOWBALL" and math.hypot(*f.action["move"]) < 0.1))

    group = "geometry, roofs and trees"
    case(group, "grounded match on supported roof", roof([0, 0, 12]),
         last(lambda f: f.trace["on_roof"] is True), "roof")
    case(group, "airborne at the same roof height", roof([0, 0, 12], grounded=False),
         last(lambda f: f.trace["on_roof"] is False), "roof")
    case(group, "height alone cannot create a roof over open terrain", roof([60, 0, 12]),
         last(lambda f: f.trace["on_roof"] is False), "roof")
    case(group, "ground under a walkable bridge is distinct from its deck", roof([0, 0, 0]),
         last(lambda f: f.trace["on_roof"] is False), "bridge")
    rooftop_loot = obs(pickups=pickup(x=15, z=5))
    larger = deepcopy(rooftop_loot)
    larger["self"].update(cubes=10)
    larger["self"]["stats"].update(radius=3.75, height=13.5, muzzle_height=10.5, eye_height=12)
    case(group, "upgraded body cannot use a narrow roof landing",
         [rooftop_loot, later(larger, 2)],
         lambda fs: fs[0].trace["destination"][2] == 5 and fs[1].trace["destination"][2] == 0
                    and not fs[1].action["jump"], "narrow")
    case(group, "concave roof bounding box excludes its empty notch", roof([7, 7, 12]),
         last(lambda f: f.trace["on_roof"] is False), "concave")
    case(group, "solid wall prevents a shot at a reported target", obs(actors=[mech()]),
         last(lambda f: not any(f.action["fire"])), "wall")
    case(group, "clearance below raised geometry permits a shot", obs(actors=[mech()]),
         last(lambda f: fired(f)), "high_beam")
    case(group, "nearby target inside foliage still permits a clear shot", obs(actors=[mech()]),
         last(lambda f: fired(f)), "canopy")
    east = mech(x=23, y=16.5)
    north = mech(x=16.5, y=23, actor_id=5)
    edge = {"pos": [16.5, 16.5, 12]}
    case(group, "second opponent exposes a rooftop peek and triggers withdrawal",
         [obs(actors=[east], self_fields=edge), later(obs(actors=[east, north], self_fields=edge))],
         lambda fs: [f.trace["exposed"] for f in fs] == [1, 2]
                    and fs[1].trace["movement_reason"] == "roof-cover"
                    and math.hypot(*fs[1].trace["destination"][:2]) < math.hypot(*edge["pos"][:2]), "roof")

    group = "regeneration and rotation"
    damaged = obs(self_fields={"hp": 400})
    almost = later(obs(self_fields={"hp": 1330}), 8)
    healed = later(obs(self_fields={"hp": 1400}, pickups=pickup(x=20)), 15)
    case(group, "safe regeneration persists at 95 percent", [damaged, almost], is_mode("REGEN"))
    case(group, "full HP ends recovery", [damaged, almost, healed], is_mode("SNOWBALL"))
    case(group, "finishable isolated opponent can interrupt recovery",
         [damaged, later(obs(actors=[mech(hp=40)], self_fields={"hp": 800}))], is_mode("COMBAT"))
    case(group, "storm pressure interrupts safe healing",
         [damaged, later(obs(outside=True, self_fields={"hp": 390}, zone_fields={"dps": 160}))],
         is_mode("ROTATE"))
    case(group, "healthy bot can finish a weak rival near low-damage boundary",
         obs(actors=[mech(hp=20)], outside=True, zone_fields={"dps": 20}), is_mode("COMBAT"))
    case(group, "weak bot returns when low-damage exposure exceeds its HP",
         obs(actors=[mech(hp=20)], outside=True, self_fields={"hp": 60},
             zone_fields={"center": [140, 0], "next_center": [140, 0], "dps": 60}), is_mode("ROTATE"))
    case(group, "storm route preserves inward progress with rival beyond it",
         obs(actors=[mech(x=14)], outside=True, zone_fields={"dps": 160}),
         last(lambda f: mode(f) == "ROTATE" and f.action["move"][0] > 0))
    case(group, "sheltered partial magazine starts reloading",
         obs(self_fields={"hp": 400, "weapons": [gun(ammo=7), None]}),
         last(lambda f: f.action.get("reload", False)))
    case(group, "sheltered full magazine stays ready",
         obs(self_fields={"hp": 400}), last(lambda f: not f.action.get("reload", False)))
    case(group, "safe healing suppresses crate firing",
         obs(crates=[{"id": 1001, "pos": [20, 0, 0], "hp": 120}], self_fields={"hp": 400}),
         last(lambda f: mode(f) == "REGEN" and not any(f.action["fire"])))

    group = "loot and observation events"
    case(group, "take visible weapon within pickup range", obs(pickups=pickup()),
         last(lambda f: f.action.get("take") == 8001))
    case(group, "cannot collect a pickup one floor above", obs(pickups=pickup(z=12)),
         last(lambda f: f.action.get("take") is None))
    case(group, "superior pickup replaces a weaker held secondary",
         obs(pickups=pickup(), self_fields={"weapons": [gun(), weapon("mortar")]}),
         last(lambda f: f.action.get("take") == 8001 and f.action.get("drop") == 1))
    case(group, "avoid taking an already held weapon",
         obs(pickups=pickup(), self_fields={"weapons": [gun(), weapon("lightning")]}),
         last(lambda f: f.action.get("take") is None))
    upgrades = {u["id"]: {key: u[key] for key in ("id", "name", "desc")} for u in hello["config"]["upgrades"]}
    case(group, "choose nanites from official offer choices",
         obs(offer={"id": 1, "expires": 15, "choices": [upgrades[k] for k in ("servos", "nanites", "sensors")]}),
         last(lambda f: f.action.get("pick") == 1))
    laser = obs(actors=[mech(x=65)], self_fields=arm("laser"))
    shorted = deepcopy(laser)
    shorted["events"] = [{"type": "effect", "effect": "short", "by": 4}]
    case(group, "incoming short suppresses laser charging",
         [shorted, later(laser, 1.9)], lambda fs: all(not fired(f, 1) for f in fs))
    case(group, "laser charging resumes after short expires",
         [shorted, later(laser, 2.2)], last(lambda f: fired(f, 1)))
    hit = deepcopy(laser)
    hit["events"] = [{"type": "hit", "target": 4, "weapon": "lightning", "amount": 60, "effect": "short"}]
    case(group, "outgoing short leaves our laser available", hit, last(lambda f: fired(f, 1)))
    case(group, "rear armor turns toward an incoming unseen hit",
         obs(events=[{"type": "damaged", "amount": 45, "weapon": "gun", "by": 8, "rear": True, "bearing": 2.2}]),
         last(lambda f: yaw_error(f, 2.2) < 0.2))
    case(group, "kill clears the sole rival and exposes collectible cubes",
         [obs(actors=[mech(hp=45)]), later(obs(events=[{"type": "kill", "target": 4}],
                                              cubes=[{"id": 5001, "pos": [16, 0, 0]}]))],
         last(lambda f: mode(f) == "SNOWBALL" and f.action["move"][0] > 0))

    group = "airborne movement and weapons"
    def airborne(x=40, **self_updates):
        me = dict(pos=[0, 0, 10], grounded=False, vel=[0, 0, 0], aim=[0, math.atan2(-2.5, x)])
        me.update(self_updates)
        return obs(actors=[mech(x=x, z=10)], self_fields=me)

    case(group, "clear aligned gun shot during a jump", airborne(),
         last(lambda f: fired(f) and f.trace["airborne_shot"]))
    case(group, "airborne weapon cooldown suppresses the shot",
         airborne(weapons=[gun(cooldown=0.9), None]),
         last(lambda f: not fired(f) and not f.trace["airborne_shot"]))
    case(group, "defensive jump and aligned gun fire on the same decision",
         obs(actors=[dict(mech(), charging=True, weapons=["gun", "laser"],
                          aim=[math.pi, math.atan2(-2.5, 40)])],
             events=[{"type": "damaged", "amount": 45, "weapon": "gun", "by": 4, "bearing": 0}]),
         last(lambda f: f.action["jump"] and fired(f) and f.trace["airborne_shot"]))
    eave_target = mech(z=8)
    def pass_eave(z, vertical_speed, seconds):
        pitch = math.atan2(8 + 4.5 - (z + vertical_speed * 0.05 + 7), 40)
        return later(obs(actors=[eave_target], self_fields={
            "grounded": False, "pos": [0, 0, z], "vel": [0, 0, vertical_speed], "aim": [0, pitch]
        }), seconds)
    case(group, "ascent clears an eave and descent blocks gunfire again",
         [pass_eave(1, 9, 0), pass_eave(8, 3, 0.8), pass_eave(1, -9, 1.9)],
         lambda fs: [fired(f) for f in fs] == [False, True, False] and fs[1].trace["airborne_shot"], "eave")
    case(group, "large airborne turret error delays the trigger", airborne(aim=[1.0, 0]),
         last(lambda f: not fired(f) and yaw_error(f, 0) < 0.1))
    case(group, "small airborne turret error can settle by first simulated tick",
         airborne(aim=[0.09, math.atan2(-2.5, 40)]), last(lambda f: fired(f)))
    case(group, "airborne lightning with sufficient jet reserve",
         airborne(**arm("lightning")), last(lambda f: fired(f, 1) and f.trace["airborne_shot"]))
    case(group, "airborne lightning saves energy required for landing",
         airborne(**arm("lightning", energy=5)), last(lambda f: not fired(f, 1) and
             f.observation["self"]["energy"] - f.trace.get("jet_energy_reserve", 0) < 3))
    case(group, "airborne laser charge is distinct from a damaging release",
         airborne(x=70, **arm("laser")), last(lambda f: fired(f, 1) and not f.trace["airborne_shot"]))
    charged = arm("laser")
    charged["weapons"][1]["charge"] = 0.8
    case(group, "aligned full-charge laser releases during flight",
         airborne(x=70, **charged), last(lambda f: not fired(f, 1) and f.trace["airborne_shot"]
                                        and "laser" in f.trace["weapon"]))
    return cases


def make_map(name, directory):
    layouts = {
        "open": [], "canopy": [],
        "narrow": [wall(12, -14, 18, 14, 5)],
        "roof": [wall(-20, -20, 20, 20, 12)],
        "wall": [wall(16, -6, 20, 6, 18)],
        "eave": [wall(14, -8, 18, 8, 12)],
        "bridge": [dict(wall(-20, -20, 20, 20, 18), base=15)],
        "high_beam": [dict(wall(15, -10, 22, 10, 22), base=16)],
        "concave": [{"base": 0, "top": 12, "ring": [-16,-16,16,-16,16,0,0,0,0,16,-16,16]}],
    }
    write_arena(directory, layouts[name])
    if name == "canopy":
        path = directory / "v2/world.json"
        world = json.loads(path.read_text(encoding="utf-8"))
        world["rules"] = {"canopyDensity": 1, "canopyBlockT": 0.25}
        world["trees"] = {"fields": ["x","y","z","height","crownR","crownBase","trunkR"],
                          "data": [20, 4, 0, 14, 10, 1, .4]}
        path.write_text(json.dumps(world), encoding="utf-8")


def run_scenarios(kit, image, hello, sample, end):
    cases = fixtures(hello, sample)
    schema = runpy.run_path(str(kit / "check/check.py"))["Schema"](kit / "mechfight.schema.json")
    groups = Counter(c.group for c in cases)
    assert len(groups) == 5 and set(groups.values()) == {10} and len({c.name for c in cases}) == 50
    by_map = defaultdict(list)
    for case in cases:
        for observation in case.observations:
            errors = schema.errors(observation, "Observation")
            assert not errors, (case.name, errors)
        by_map[case.map_name].append(case)
    failed = []
    for name, map_cases in by_map.items():
        diagnostics, runs = [], []
        with TemporaryDirectory() as folder:
            arena = Path(folder)
            make_map(name, arena)
            with docker_bot(image, arena, diagnostics, trace=True) as (send, receive):
                for case in map_cases:
                    start_bot(send, receive, hello)
                    runs.append([ask(send, receive, o) for o in case.observations])
                send(end)
        traces = [json.loads(line) for line in diagnostics if line.strip()]
        assert len(traces) == sum(map(len, runs)), (name, "expected one trace per action", len(traces))
        iterator = iter(traces)
        for case, actions in zip(map_cases, runs):
            frames = [Frame(o, a, next(iterator)) for o, a in zip(case.observations, actions)]
            for f in frames:
                errors = schema.errors(f.action, "Action")
                assert not errors, (case.name, errors)
                assert f.trace["tick"] == f.observation["tick"] and f.trace["mode"] in MODES, f.trace
                assert isinstance(f.trace["movement_reason"], str) and f.trace["movement_reason"], f.trace
                assert len(f.trace["destination"]) == 3 and all(map(math.isfinite, f.trace["destination"])), f.trace
                assert isinstance(f.trace["airborne_shot"], bool) and "weapon" in f.trace, f.trace
                assert math.hypot(*f.action["move"]) <= 1.000001 and all(map(math.isfinite, f.action["aim"])), f.action
            if case.check(frames):
                print("PASS", case.group, "-", case.name)
            else:
                failed.append(case.name)
                print("FAIL", case.group, "-", case.name, [(f.action, f.trace) for f in frames])
    print(f"{len(cases) - len(failed)}/{len(cases)} named mechanism scenarios passed")
    return not failed


def compare_replay(kit, image, baseline, hello, observations, end, report_path):
    before = replay(baseline, kit / "arena", hello, observations, end)
    diagnostics = []
    after = replay(image, kit / "arena", hello, observations, end, diagnostics, trace=True)
    traces = [json.loads(line) for line in diagnostics]
    assert [t["tick"] for t in traces] == [o["tick"] for o in observations]
    changes = {key: sum(a.get(key) != b.get(key) for a, b in zip(before, after))
               for key in ("move", "fire", "boost", "jump", "reload", "take", "pick")}

    def movement_counts(actions):
        directions = [a.get("move", [0, 0]) for a in actions]
        lengths = [math.hypot(*move) for move in directions]
        counts = {"idle_commands": sum(length < 0.1 for length in lengths),
                  "straight_pairs": 0, "reversals": 0}
        for previous, current, a, b in zip(directions, directions[1:], lengths, lengths[1:]):
            if min(a, b) < 0.1:
                continue
            cosine = sum(x * y for x, y in zip(previous, current)) / (a * b)
            counts["straight_pairs"] += cosine >= math.cos(math.radians(15))
            counts["reversals"] += cosine <= math.cos(math.radians(135))
        return counts

    movement = {"baseline": movement_counts(before), "candidate": movement_counts(after)}
    print("Recorded action changes vs", baseline, changes)
    print("Recorded movement command counts:", movement)
    print("Recorded intents:", dict(Counter(t["mode"] for t in traces)))
    print("Movement reasons:", dict(Counter(t["movement_reason"] for t in traces)))
    print("Baseline vs candidate trigger observations:",
          sum(any(a["fire"]) for a in before), sum(any(a["fire"]) for a in after))
    print("Candidate airborne firing decisions:", sum(t["airborne_shot"] for t in traces))
    if report_path:
        report_path.write_text(json.dumps({"baseline": baseline, "candidate": image, "changes": changes,
                                          "baseline_actions": before, "actions": after, "traces": traces,
                                          "movement_counts": movement}),
                               encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kit", type=Path)
    parser.add_argument("image")
    parser.add_argument("--baseline", help="also compare actions on the official recorded stream")
    parser.add_argument("--replay-only", action="store_true", help="skip synthetic scenarios for a staged comparison")
    parser.add_argument("--report", type=Path, help="save recorded actions and per-observation traces")
    args = parser.parse_args()
    hello, observations, end = load_recording(args.kit)
    if (args.replay_only or args.report) and not args.baseline:
        parser.error("--replay-only and --report require a --baseline image")
    passed = args.replay_only or run_scenarios(args.kit, args.image, hello, observations[0], end)
    if args.baseline:
        compare_replay(args.kit, args.image, args.baseline, hello, observations, end, args.report)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
