#!/usr/bin/env python3
"""Coleta uma baseline leve do robô sem alterar os loops da missão."""

import argparse
import base64
import csv
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import select
import shutil
import socket
import struct
import subprocess
import time
from urllib.request import urlopen


DEFAULT_DURATION_SECONDS = 60.0
DEFAULT_INTERVAL_SECONDS = 1.0
TELEMETRY_WINDOW_SECONDS = 5.0
THROTTLING_INTERVAL_SECONDS = 5.0


def read_text(path):
    try:
        return Path(path).read_text(encoding="utf-8")
    except OSError:
        return ""


def read_system_cpu_ticks():
    line = read_text("/proc/stat").splitlines()
    if not line or not line[0].startswith("cpu "):
        return None
    try:
        values = [int(value) for value in line[0].split()[1:]]
    except ValueError:
        return None
    if len(values) < 4:
        return None
    idle_ticks = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle_ticks


def read_process_cpu_ticks(pid):
    stat = read_text(f"/proc/{pid}/stat")
    command_end = stat.rfind(")")
    if command_end < 0:
        return None
    fields = stat[command_end + 2:].split()
    try:
        # Após o nome entre parênteses, o índice zero corresponde ao campo 3.
        return int(fields[11]) + int(fields[12])
    except (IndexError, ValueError):
        return None


def read_process_rss_mb(pid):
    for line in read_text(f"/proc/{pid}/status").splitlines():
        if line.startswith("VmRSS:"):
            try:
                return int(line.split()[1]) / 1024.0
            except (IndexError, ValueError):
                return None
    return None


def process_command(pid):
    command = read_text(f"/proc/{pid}/cmdline")
    return command.replace("\x00", " ").strip()


def find_processes():
    result = {"robot": None, "vision": None, "forward": None}
    proc = Path("/proc")
    if not proc.is_dir():
        return result
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        command = process_command(pid)
        if not command:
            continue
        arguments = command.split()
        if any(Path(argument).name in ("robot_test", "robot_test.exe")
               for argument in arguments):
            result["robot"] = pid
        elif "forward_camera_stream.py" in command:
            result["forward"] = pid
        elif "camera_line_frame.py" in command:
            result["vision"] = pid
    return result


def read_memory():
    values = {}
    for line in read_text("/proc/meminfo").splitlines():
        if ":" not in line:
            continue
        name, raw_value = line.split(":", 1)
        try:
            values[name] = int(raw_value.split()[0])
        except (IndexError, ValueError):
            continue
    total_kb = values.get("MemTotal")
    available_kb = values.get("MemAvailable")
    if not total_kb or available_kb is None:
        return None, None
    used_kb = total_kb - available_kb
    return used_kb / 1024.0, used_kb * 100.0 / total_kb


def read_temperature_celsius():
    text = read_text("/sys/class/thermal/thermal_zone0/temp").strip()
    try:
        return float(text) / 1000.0
    except ValueError:
        return None


def read_load_average():
    fields = read_text("/proc/loadavg").split()
    try:
        return float(fields[0]), float(fields[1]), float(fields[2])
    except (IndexError, ValueError):
        return None, None, None


class CpuSampler:
    """Calcula deltas sem criar outro polling dentro dos processos do robô."""

    def __init__(self):
        self.previous_system = None
        self.previous_process = {}
        self.cpu_count = max(1, os.cpu_count() or 1)

    def sample(self, processes):
        current_system = read_system_cpu_ticks()
        current_process = {
            role: read_process_cpu_ticks(pid) if pid is not None else None
            for role, pid in processes.items()
        }
        system_cpu = None
        process_cpu = {role: None for role in processes}
        if current_system is not None and self.previous_system is not None:
            total_delta = current_system[0] - self.previous_system[0]
            idle_delta = current_system[1] - self.previous_system[1]
            if total_delta > 0:
                system_cpu = 100.0 * (1.0 - idle_delta / total_delta)
                for role, ticks in current_process.items():
                    previous = self.previous_process.get(role)
                    if ticks is not None and previous is not None:
                        process_cpu[role] = (
                            (ticks - previous) * self.cpu_count * 100.0
                            / total_delta
                        )
        self.previous_system = current_system
        self.previous_process = current_process
        return system_cpu, process_cpu


class ThrottlingSampler:
    """Consulta o firmware no máximo uma vez a cada cinco segundos."""

    def __init__(self):
        self.command = shutil.which("vcgencmd")
        self.last_sample_at = 0.0
        self.cached = "indisponível"

    def sample(self, now):
        if not self.command:
            return self.cached
        if now - self.last_sample_at < THROTTLING_INTERVAL_SECONDS:
            return self.cached
        self.last_sample_at = now
        try:
            result = subprocess.run(
                [self.command, "get_throttled"],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
            if result.returncode == 0 and result.stdout.strip():
                self.cached = result.stdout.strip().replace("throttled=", "")
            else:
                self.cached = "erro"
        except (OSError, subprocess.SubprocessError):
            self.cached = "erro"
        return self.cached


class TelemetryWebSocket:
    """Recebe frames do WebSocket usando somente a biblioteca padrão."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.connection = None
        self.buffer = bytearray()
        self.received_at = deque()
        self.latest = {}
        self.latest_payload_bytes = 0
        self.latest_transport_age_ms = None
        self.last_sequence = None
        self.dropped_messages = 0

    def connect(self):
        key = base64.b64encode(secrets.token_bytes(16)).decode("ascii")
        connection = socket.create_connection((self.host, self.port), timeout=2.0)
        request = (
            "GET /ws HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        connection.sendall(request.encode("ascii"))
        response = bytearray()
        while b"\r\n\r\n" not in response:
            chunk = connection.recv(4096)
            if not chunk:
                raise ConnectionError("O servidor fechou o handshake WebSocket.")
            response.extend(chunk)
        headers, remaining = response.split(b"\r\n\r\n", 1)
        if b" 101 " not in headers.split(b"\r\n", 1)[0]:
            raise ConnectionError("O dashboard recusou o WebSocket.")
        expected = base64.b64encode(hashlib.sha1(
            (key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")
        ).digest())
        if expected not in headers:
            raise ConnectionError("A resposta WebSocket não passou na validação.")
        connection.setblocking(False)
        self.connection = connection
        self.buffer.extend(remaining)
        self._parse_frames()

    def poll(self, timeout_seconds):
        if self.connection is None:
            time.sleep(timeout_seconds)
            return
        readable, _, _ = select.select(
            [self.connection], [], [], max(0.0, timeout_seconds)
        )
        if not readable:
            return
        try:
            chunk = self.connection.recv(65536)
        except BlockingIOError:
            return
        if not chunk:
            self.close()
            return
        self.buffer.extend(chunk)
        self._parse_frames()

    def _parse_frames(self):
        while len(self.buffer) >= 2:
            first, second = self.buffer[0], self.buffer[1]
            opcode = first & 0x0F
            masked = bool(second & 0x80)
            length = second & 0x7F
            offset = 2
            if length == 126:
                if len(self.buffer) < 4:
                    return
                length = struct.unpack("!H", self.buffer[2:4])[0]
                offset = 4
            elif length == 127:
                if len(self.buffer) < 10:
                    return
                length = struct.unpack("!Q", self.buffer[2:10])[0]
                offset = 10
            mask = None
            if masked:
                if len(self.buffer) < offset + 4:
                    return
                mask = self.buffer[offset:offset + 4]
                offset += 4
            if len(self.buffer) < offset + length:
                return
            payload = bytearray(self.buffer[offset:offset + length])
            del self.buffer[:offset + length]
            if mask is not None:
                for index in range(len(payload)):
                    payload[index] ^= mask[index % 4]
            if opcode == 0x8:
                self.close()
                return
            if opcode != 0x1:
                continue
            try:
                self.latest = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            try:
                sequence = int(self.latest.get("telemetrySequence"))
                if (self.last_sequence is not None
                        and sequence > self.last_sequence + 1):
                    self.dropped_messages += sequence - self.last_sequence - 1
                self.last_sequence = sequence
            except (TypeError, ValueError):
                pass
            try:
                generated_ms = float(self.latest.get("telemetryGeneratedUnixMs"))
                self.latest_transport_age_ms = time.time() * 1000.0 - generated_ms
            except (TypeError, ValueError):
                self.latest_transport_age_ms = None
            now = time.monotonic()
            self.received_at.append(now)
            self.latest_payload_bytes = len(payload)
            self._discard_old_receipts(now)

    def _discard_old_receipts(self, now):
        minimum = now - TELEMETRY_WINDOW_SECONDS
        while self.received_at and self.received_at[0] < minimum:
            self.received_at.popleft()

    def snapshot(self):
        now = time.monotonic()
        self._discard_old_receipts(now)
        effective_hz = None
        if len(self.received_at) >= 2:
            elapsed = self.received_at[-1] - self.received_at[0]
            if elapsed > 0.0:
                effective_hz = (len(self.received_at) - 1) / elapsed
        generated_ms = self.latest.get("telemetryGeneratedUnixMs")
        try:
            age_ms = time.time() * 1000.0 - float(generated_ms)
        except (TypeError, ValueError):
            age_ms = None
        return {
            "telemetryHz": effective_hz,
            "telemetryAgeMs": age_ms,
            "telemetryTransportAgeMs": self.latest_transport_age_ms,
            "telemetryDroppedMessages": self.dropped_messages,
            "telemetryServerIntervalMs": self.latest.get(
                "telemetryEffectiveIntervalMs"
            ),
            "telemetryServerWorkMs": self.latest.get("telemetryServerWorkMs"),
            "websocketClients": self.latest.get("websocketClientCount"),
            "telemetryPayloadBytes": self.latest_payload_bytes or None,
        }

    def close(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except OSError:
                pass
        self.connection = None


def read_camera_status(host, port):
    try:
        with urlopen(
            f"http://{host}:{port}/camera-status.json", timeout=1.0
        ) as response:
            status = json.load(response)
    except (OSError, ValueError):
        return None, None
    try:
        fps = float(status.get("fps"))
    except (TypeError, ValueError):
        fps = None
    try:
        age_ms = time.time() * 1000.0 - float(status.get("timestamp")) * 1000.0
    except (TypeError, ValueError):
        age_ms = None
    return fps, age_ms


def number(value, decimals=3):
    if value is None:
        return ""
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return ""
    return round(numeric, decimals) if math.isfinite(numeric) else ""


def percentile(values, ratio):
    ordered = sorted(value for value in values if value is not None)
    if not ordered:
        return None
    index = max(0, math.ceil(len(ordered) * ratio) - 1)
    return ordered[index]


def summarize(rows, columns):
    summary = {"samples": len(rows)}
    for column in columns:
        values = []
        for row in rows:
            try:
                value = float(row[column])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                values.append(value)
        summary[column] = {
            "mean": sum(values) / len(values) if values else None,
            "p95": percentile(values, 0.95),
            "max": max(values) if values else None,
        }
    return summary


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Coleta CPU, RAM, câmera e telemetria em baixa frequência."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_SECONDS)
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_SECONDS)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    if arguments.duration <= 0.0:
        parser.error("--duration deve ser maior que zero.")
    if arguments.interval < 1.0:
        parser.error("--interval não pode ser menor que 1 segundo.")
    if arguments.output is None:
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        arguments.output = Path(f"performance-baseline-{timestamp}.csv")
    return arguments


def main():
    arguments = parse_arguments()
    fieldnames = [
        "timestampUtc", "elapsedSeconds", "systemCpuPercent", "load1",
        "load5", "load15", "ramUsedMb", "ramUsedPercent",
        "temperatureCelsius", "throttledHex", "robotPid",
        "robotCpuPercent", "robotRssMb", "visionPid",
        "visionCpuPercent", "visionRssMb", "forwardPid",
        "forwardCpuPercent", "forwardRssMb", "cameraFps",
        "cameraStatusAgeMs", "telemetryHz", "telemetryAgeMs",
        "telemetryTransportAgeMs", "telemetryDroppedMessages",
        "telemetryServerIntervalMs", "telemetryServerWorkMs",
        "websocketClients", "telemetryPayloadBytes",
    ]
    summary_columns = [
        "systemCpuPercent", "ramUsedMb", "temperatureCelsius",
        "robotCpuPercent", "robotRssMb", "visionCpuPercent", "visionRssMb",
        "cameraFps", "cameraStatusAgeMs", "telemetryHz", "telemetryAgeMs",
        "telemetryTransportAgeMs", "telemetryDroppedMessages",
        "telemetryServerIntervalMs", "telemetryServerWorkMs",
        "telemetryPayloadBytes",
    ]
    cpu_sampler = CpuSampler()
    throttling_sampler = ThrottlingSampler()
    websocket = TelemetryWebSocket(arguments.host, arguments.port)
    try:
        websocket.connect()
        websocket_state = "conectado"
    except (OSError, ConnectionError) as error:
        websocket_state = f"indisponível: {error}"
    rows = []
    started_at = time.monotonic()
    next_sample_at = started_at
    end_at = started_at + arguments.duration
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    print(
        f"Baseline iniciada por {arguments.duration:.0f}s em "
        f"{arguments.host}:{arguments.port}; WebSocket {websocket_state}."
    )
    try:
        with arguments.output.open("w", encoding="utf-8", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=fieldnames)
            writer.writeheader()
            while time.monotonic() < end_at:
                now = time.monotonic()
                if now < next_sample_at:
                    websocket.poll(min(0.5, next_sample_at - now, end_at - now))
                    continue
                processes = find_processes()
                system_cpu, process_cpu = cpu_sampler.sample(processes)
                ram_used_mb, ram_used_percent = read_memory()
                load1, load5, load15 = read_load_average()
                camera_fps, camera_age_ms = read_camera_status(
                    arguments.host, arguments.port
                )
                telemetry = websocket.snapshot()
                row = {
                    "timestampUtc": datetime.now(timezone.utc).isoformat(),
                    "elapsedSeconds": number(now - started_at),
                    "systemCpuPercent": number(system_cpu),
                    "load1": number(load1),
                    "load5": number(load5),
                    "load15": number(load15),
                    "ramUsedMb": number(ram_used_mb),
                    "ramUsedPercent": number(ram_used_percent),
                    "temperatureCelsius": number(read_temperature_celsius()),
                    "throttledHex": throttling_sampler.sample(now),
                    "robotPid": processes["robot"] or "",
                    "robotCpuPercent": number(process_cpu["robot"]),
                    "robotRssMb": number(
                        read_process_rss_mb(processes["robot"])
                        if processes["robot"] else None
                    ),
                    "visionPid": processes["vision"] or "",
                    "visionCpuPercent": number(process_cpu["vision"]),
                    "visionRssMb": number(
                        read_process_rss_mb(processes["vision"])
                        if processes["vision"] else None
                    ),
                    "forwardPid": processes["forward"] or "",
                    "forwardCpuPercent": number(process_cpu["forward"]),
                    "forwardRssMb": number(
                        read_process_rss_mb(processes["forward"])
                        if processes["forward"] else None
                    ),
                    "cameraFps": number(camera_fps),
                    "cameraStatusAgeMs": number(camera_age_ms),
                    **{key: number(value) for key, value in telemetry.items()},
                }
                writer.writerow(row)
                rows.append(row)
                next_sample_at += arguments.interval
                if next_sample_at <= now:
                    next_sample_at = now + arguments.interval
    finally:
        websocket.close()

    summary = summarize(rows, summary_columns)
    summary["durationSeconds"] = arguments.duration
    summary["intervalSeconds"] = arguments.interval
    summary["dashboardHost"] = arguments.host
    summary_path = arguments.output.with_suffix(".summary.json")
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"CSV: {arguments.output.resolve()}")
    print(f"Resumo: {summary_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
