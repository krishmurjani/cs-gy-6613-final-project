FROM python:3.9-slim


ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1


WORKDIR /app


COPY . /app


RUN apt-get update && apt-get install -y \
    curl \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


RUN pip install --upgrade pip
RUN pip install -r requirements.txt


CMD ["python", "main.py"]