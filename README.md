# Mech Royale

Survival-first bot for the Builders Buildathon. It starts rotating early when the route is clear. While threatened, it chooses an escape in the current safe zone until the shrinking boundary makes rotation urgent. It scans while travelling and returns fire while withdrawing when an enemy aims at it with a clear shot. It uses solid structures for cover and reserves short single-shot peeks for safe situations.

An exposed enemy laser charge triggers lateral evasion. Short defensive jumps are checked against the ceiling, terrain, landing space, and moving zone. The bot can also fire on a participant in a confirmed fight between other mechs while keeping its current survival movement.

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
