from goseek-base:latest

WORKDIR /goseek-challenge

COPY baselines/agents.py baselines/agents.py

COPY baselines/config/random-agent.yaml agent.yaml
