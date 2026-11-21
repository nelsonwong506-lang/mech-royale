# Mech Royale

Placement-focused C++20 bot for the Builders Buildathon. Each observation selects a destination for one of four modes. COMBAT compares expected damage traded at reachable positions. REGEN seeks enough time in cover to recover to 98% HP. SNOWBALL looks for weapons, cubes and unexplored roofs. ROTATE follows a route to the safe zone.

At startup the bot indexes roof interiors, edges, nearby landings and canopy geometry. During a match it checks the current mech size, energy, zone and reported opponents before moving. The legs and weapons act independently, including on takeoff and in flight. Lightning shares energy with the jets and a charging laser affects movement.

Build the competition image from the repository root:

```sh
docker build --platform linux/amd64 -t mech-royale .
```

Download and unpack the [official bot kit](https://www.joinbuilders.org/buildathon/tracks/builders/files/mechfight-kit.tar.gz). From the extracted kit directory in a Linux shell, run the official checker:

```sh
python3 check/check.py --map-dir /arena -- docker run --rm -i \
  --network none --cpus 1 --memory 512m --memory-swap 512m --pids-limit 128 \
  --cap-drop ALL --read-only --tmpfs /tmp:rw,size=64m --user 65534:65534 \
  --mount "type=bind,source=$PWD/arena,target=/arena,readonly" mech-royale
```

The checker replays recorded observations. Submit the public repository URL on the [Mech Royale track page](https://www.joinbuilders.org/buildathon/tracks/builders) to get a practice score.

Run the mechanism scenarios and the existing firing regressions from the repository root. Replace `/path/to/mechfight-kit` with the unpacked kit directory.

```sh
python3 tests/vnext_scenarios.py /path/to/mechfight-kit mech-royale
python3 tests/firing_replay.py /path/to/mechfight-kit mech-royale
python3 tests/damage_and_exploration.py /path/to/mechfight-kit mech-royale
```

For a paired comparison, add `--baseline OLD_IMAGE --report actions.json` to the first command. The comparison runs both images on the same 300 observations. It records the chosen mode, movement reason, destination and airborne firing decisions. Local tools can also request this per-observation JSON on stderr by setting `MECH_TRACE=1` in the bot container.

## Reinforcement-learning reward design

With an interactive simulator, train an agent to choose among safe rotation, withdrawal, and cover targets. Keep the movement and fire checks in the C++ controller as constraints on those choices. At the end of each episode, use:

```text
reward = (16 - place)
       + 0.25 × fraction_of_match_survived
       - 0.25 × fraction_of_match_spent_outside_zone
       - 0.10 × fraction_of_match_exposed_to_aimed_enemy_fire
```

Clamp each fraction to `[0, 1]` using the configured match duration. The secondary terms can change the reward by at most 0.60 points between episodes, so one finishing place is worth more than all secondary terms combined. Eliminations have no direct bonus. The recorded observations in the kit always replay the same positions and outcomes, so training this policy requires an environment that responds to the bot's actions.
