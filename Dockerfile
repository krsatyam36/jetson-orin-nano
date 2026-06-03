FROM nvcr.io/nvidia/l4t-jetpack:r36.3.0

WORKDIR /app

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    espeak \
    alsa-utils \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    && rm -rf /var/lib/apt/lists/*

# Python deps
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Source
COPY . .

# Health check
EXPOSE 9090
HEALTHCHECK --interval=30s --timeout=5s --retries=3 \
    CMD python3 -c "import urllib.request; urllib.request.urlopen('http://localhost:9090/health/ping')"

# Default: run health endpoint
CMD ["python3", "drone/health.py", "--port", "9090"]
