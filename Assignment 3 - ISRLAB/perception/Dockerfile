FROM python:3.10-slim
LABEL authors="giodegas"

COPY *.py .
COPY requirements.txt .
RUN pip install -r requirements.txt

EXPOSE 8100
CMD ["python", "perception.py"]
