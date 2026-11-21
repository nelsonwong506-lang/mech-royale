# Mech Royale

Placement-focused C++20 bot for the Builders Buildathon. It rotates early into the next safe zone, explicitly picks up and upgrades secondary weapons (`take`/`drop`, prioritizing lightning > laser > mortar), uses each weapon's real mechanics (30° auto-hit lightning cones with chain clustering, charge-and-release laser with mortar cook-off, and low/high-arc mortar over cover), and snowballs up to 10 power cubes via crates, NPC drones, and supply drops.

In multi-enemy engagements, it dynamically repositions and cover-splits to isolate 1v1 angles, keeps exposed threats inside its 180° front armor arc to prevent 1.5× rear-armor hits, and holds fire behind cover when low HP to trigger out-of-combat regeneration. It handles rooftop drop-offs, wide-building A* routing without thrashing, and 2D grid-DDA raycasting with strict per-tick time budgets well under the 50 ms limit.

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

## Reinforcement-learning reward design

With an interactive simulator, train an agent to choose among safe rotation, withdrawal, and cover targets. Keep the movement and fire checks in the C++ controller as constraints on those choices. At the end of each episode, use:

```text
reward = (16 - place)
       + 0.25 × fraction_of_match_survived
       - 0.25 × fraction_of_match_spent_outside_zone
       - 0.10 × fraction_of_match_exposed_to_aimed_enemy_fire
```

Clamp each fraction to `[0, 1]` using the configured match duration. The secondary terms can change the reward by at most 0.60 points between episodes, so one finishing place is worth more than all secondary terms combined. Eliminations have no direct bonus. The recorded observations in the kit always replay the same positions and outcomes, so training this policy requires an environment that responds to the bot's actions.
