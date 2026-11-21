FROM debian:bookworm-slim AS compiler
RUN apt-get update \
    && apt-get install -y --no-install-recommends g++ nlohmann-json3-dev=3.11.2-2 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY cpp/ ./
RUN g++ -std=c++20 -O3 -DNDEBUG -Wall -Wextra -Wpedantic \
    -static-libstdc++ -static-libgcc main.cpp arena.cpp navigation.cpp survival.cpp tactics.cpp -o /mech-royale

FROM debian:bookworm-slim
COPY --from=compiler /mech-royale /usr/local/bin/mech-royale
ENTRYPOINT ["/usr/local/bin/mech-royale"]
