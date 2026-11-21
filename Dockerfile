FROM python:3.13-slim
WORKDIR /bot
COPY example_bot.py mechfight.py navigation.py arena.py tactics.py ./
ENTRYPOINT ["python3", "-u", "/bot/example_bot.py"]
