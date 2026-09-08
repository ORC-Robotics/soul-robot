#!/usr/bin/env python3
import json
import os
import platform
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


HOST = "127.0.0.1"
PORT = int(os.environ.get("OBR_DEPLOY_PANEL_PORT", "8765"))
MAX_LOG_LINES = 300

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent

deploy_running = False
deploy_exit_code = None
deploy_log = []
deploy_lock = threading.Lock()


def append_log(line):
    global deploy_log
    with deploy_lock:
        deploy_log.append(line.rstrip())
        deploy_log = deploy_log[-MAX_LOG_LINES:]


def deploy_command():
    if platform.system() == "Windows":
        return [
            "powershell",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(SCRIPT_DIR / "deploy.ps1"),
            "-Service",
        ]

    return ["bash", str(SCRIPT_DIR / "deploy.sh"), "--service"]


def run_deploy():
    global deploy_running, deploy_exit_code

    command = deploy_command()
    append_log(f"Executando: {' '.join(command)}")

    try:
        process = subprocess.Popen(
            command,
            cwd=PROJECT_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        assert process.stdout is not None
        for line in process.stdout:
            append_log(line)

        deploy_exit_code = process.wait()
        append_log(f"Deploy finalizado com código {deploy_exit_code}.")
    except Exception as error:
        deploy_exit_code = 1
        append_log(f"Erro ao executar deploy: {error}")
    finally:
        with deploy_lock:
            deploy_running = False


class DeployPanelHandler(BaseHTTPRequestHandler):
    def log_message(self, format_text, *args):
        return

    def do_GET(self):
        if self.path.startswith("/status"):
            self.send_status()
            return

        self.send_page()

    def do_POST(self):
        if self.path != "/deploy":
            self.send_response(404)
            self.end_headers()
            return

        self.start_deploy()

    def send_page(self):
        html = """<!doctype html>
<html lang="pt-BR">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>OBR Deploy</title>
  <style>
    :root { color-scheme: dark; font-family: Inter, system-ui, sans-serif; background: #101418; color: #edf2f7; }
    body { margin: 0; min-height: 100vh; display: grid; place-items: center; background: #101418; }
    main { width: min(860px, calc(100% - 32px)); }
    header { display: flex; align-items: center; justify-content: space-between; gap: 16px; margin-bottom: 18px; }
    h1 { margin: 0; font-size: 2rem; }
    button { min-height: 52px; padding: 0 18px; border: 0; border-radius: 8px; background: #4ade80; color: #071016; font-size: 1rem; font-weight: 800; cursor: pointer; }
    button:disabled { cursor: wait; opacity: 0.65; }
    .status { margin: 0 0 12px; color: #a8b3c1; font-weight: 700; }
    pre { min-height: 420px; max-height: 65vh; overflow: auto; margin: 0; padding: 16px; border: 1px solid #2d3742; border-radius: 8px; background: #151b22; white-space: pre-wrap; }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>OBR Deploy</h1>
      <button id="deployButton" type="button">Deploy automático</button>
    </header>
    <p id="status" class="status">Pronto</p>
    <pre id="log"></pre>
  </main>
  <script>
    const button = document.getElementById("deployButton");
    const statusText = document.getElementById("status");
    const log = document.getElementById("log");

    async function refreshStatus() {
      const response = await fetch("/status", { cache: "no-store" });
      const data = await response.json();
      button.disabled = data.running;
      statusText.textContent = data.running
        ? "Deploy em andamento..."
        : (data.exitCode === null ? "Pronto" : `Último deploy terminou com código ${data.exitCode}`);
      log.textContent = data.log.join("\\n");
      log.scrollTop = log.scrollHeight;
    }

    button.addEventListener("click", async () => {
      button.disabled = true;
      await fetch("/deploy", { method: "POST" });
      refreshStatus();
    });

    setInterval(refreshStatus, 1000);
    refreshStatus();
  </script>
</body>
</html>"""
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html.encode("utf-8"))))
        self.end_headers()
        self.wfile.write(html.encode("utf-8"))

    def send_status(self):
        with deploy_lock:
            status = {
                "running": deploy_running,
                "exitCode": deploy_exit_code,
                "log": deploy_log,
            }

        content = json.dumps(status).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def start_deploy(self):
        global deploy_running, deploy_exit_code, deploy_log

        with deploy_lock:
            if deploy_running:
                self.send_status()
                return

            # Mantém um único deploy por vez para evitar cópias e reinícios concorrentes.
            deploy_running = True
            deploy_exit_code = None
            deploy_log = []

        threading.Thread(target=run_deploy, daemon=True).start()
        self.send_status()


def main():
    server = ThreadingHTTPServer((HOST, PORT), DeployPanelHandler)
    print(f"Painel de deploy: http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nPainel de deploy encerrado.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
