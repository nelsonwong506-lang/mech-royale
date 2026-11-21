# Mech Royale

Python starter for the Builders Buildathon Mech Royale track. Edit `example_bot.py` to change the bot. `mechfight.py` handles the game protocol.

Build for the competition with `docker build --platform linux/amd64 -t mech-royale .`.

Download the [official kit](https://www.joinbuilders.org/buildathon/tracks/builders/files/mechfight-kit.tar.gz) for the protocol reference, conformance checker, recorded observations, and local arena. Copy `check/`, `arena/`, and `mechfight.schema.json` from the kit into this folder, then run:

```sh
python3 check/check.py -- python3 -u example_bot.py
```

See the kit README for the Docker conformance command. Submit the public repository URL on the [Mech Royale track page](https://www.joinbuilders.org/buildathon/tracks/builders).
