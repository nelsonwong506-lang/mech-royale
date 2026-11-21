# Mech Royale

Defense-first bot for the Builders Buildathon. `cpp/main.cpp` handles game messages and coordinates the C++ navigation and tactics modules in `cpp/`.

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
