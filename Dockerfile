FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY backend/ ./backend/

RUN groupadd --gid 10001 appgroup \
    && useradd --uid 10001 --gid 10001 --no-create-home --shell /usr/sbin/nologin appuser \
    && mkdir -p /app/data \
    && chown -R appuser:appgroup /app/data

USER 10001:10001

EXPOSE 8095

HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
  CMD python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8095/health', timeout=3)"

CMD ["python", "-m", "backend.app", "--host", "0.0.0.0", "--port", "8095", "--db", "/app/data/health_station.db"]
