PYTHON ?= python
PORT ?= 8095

.PHONY: help install demo run test lint up down clean

help:
	@echo "Targets:"
	@echo "  make install  Install Python dependencies"
	@echo "  make demo     Run 3-station simulator + web UI on http://127.0.0.1:$(PORT)"
	@echo "  make run      Run MQTT backend + web UI (requires Mosquitto)"
	@echo "  make test     Run unit & security test suite"
	@echo "  make lint     Run ruff linter and compileall check"
	@echo "  make up       Start Mosquitto + backend + simulator via Docker Compose"
	@echo "  make down     Stop Docker Compose services"
	@echo "  make clean    Remove caches and temporary databases"

install:
	$(PYTHON) -m pip install -r requirements.txt

demo:
	$(PYTHON) -m backend.app --demo --port $(PORT) --open-browser

run:
	$(PYTHON) -m backend.app --port $(PORT)

test:
	$(PYTHON) -m unittest discover -s tests -v

lint:
	$(PYTHON) -m compileall -q backend tests
	ruff check backend tests

up:
	docker compose --profile sim up -d --build

down:
	docker compose --profile sim down

clean:
	rm -rf __pycache__ backend/__pycache__ tests/__pycache__ .ruff_cache data/
