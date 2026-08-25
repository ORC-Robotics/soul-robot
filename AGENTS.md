````md
# OBR2026K Agent Guidelines

Before changing this repository, read this file and follow these project rules.

This project is for an OBR robot running on a Raspberry Pi, with C++ robot code and a dashboard/console for development, testing, and telemetry.

The priority order is:

1. Robot safety
2. Code simplicity
3. Clear explanations in Brazilian Portuguese
4. Reliable deploy flow
5. Useful dashboard features

Do not make the code clever. Make it understandable.

---

## Project Goals

- Keep the Raspberry Pi deploy flow simple: one command should deploy, build, and restart the robot service.
- Keep runtime dependencies minimal unless the team explicitly chooses a library.
- Favor small C++ modules with clear ownership over a large `main.cpp`.
- The dashboard is a development and testing tool; robot safety has priority over convenience.
- Code must be easy for students and team members to understand, modify, and debug.
- Prefer boring, explicit, predictable code over advanced abstractions.

---

## Language Rules

- Code identifiers, file names, class names, function names, and variable names should remain in English.
- Code comments must be written in Brazilian Portuguese.
- Documentation may be written in English or Brazilian Portuguese, but comments inside source code must be in Brazilian Portuguese.
- Comments must use correct spelling, grammar, punctuation, and accents.
- Do not write comments without accents when the correct Portuguese word requires accents.

Good examples:

```cpp
// Tempo máximo, em milissegundos, antes de parar os motores.
// Se o painel parar de enviar comandos, o robô não deve continuar andando
// com o último comando recebido.
constexpr int COMMAND_TIMEOUT_MS = 300;
````

```cpp
// Limita a potência do motor para evitar valores fora da faixa segura.
// Alterar este limite afeta diretamente a velocidade máxima do robô.
double safePower = clamp(rawPower, -1.0, 1.0);
```

Bad examples:

```cpp
// tempo maximo antes de parar motor
constexpr int COMMAND_TIMEOUT_MS = 300;
```

```cpp
// seta potencia
double safePower = clamp(rawPower, -1.0, 1.0);
```

---

## Spelling and Grammar Rules for Comments

Before adding or changing comments, review them for:

* Correct accents: `robô`, `potência`, `máximo`, `mínimo`, `saída`, `emergência`, `segurança`.
* Clear punctuation.
* Complete meaning.
* No informal abbreviations.
* No unclear slang.
* No broken Portuguese.
* No mixed Portuguese and English in the same sentence unless the English term is a technical name.

Prefer:

```cpp
// Define se a parada de emergência está ativa.
// Quando estiver ativa, todos os comandos de motor devem ser ignorados.
bool emergencyStopActive;
```

Avoid:

```cpp
// flag pra saber se estop ta ativo
bool emergencyStopActive;
```

Technical terms may remain in English when they are names of technologies, protocols, files, or modules:

```cpp
// Envia a telemetria atual para o DashboardServer via WebSocket.
```

---

## Code Style Philosophy

Write code as if a teammate with basic C++ knowledge will need to explain it during a competition.

Good code in this repository should be:

* Simple to read.
* Easy to change.
* Easy to debug.
* Split into small files with clear responsibility.
* Commented where the behavior, risk, or intention is not obvious.
* Conservative with dependencies.
* Safe by default.

Avoid:

* Overengineering.
* Generic frameworks.
* Complex templates.
* Unnecessary inheritance.
* Hidden side effects.
* Large functions that do many things.
* Large classes that own unrelated responsibilities.
* Magic numbers spread across the codebase.

---

## Commenting Rules

Comments are required in important code paths.

Comments must explain, when relevant:

* What the code does.
* Why the code exists.
* What changing it affects.
* What hardware, pin, sensor, motor, or dashboard behavior it controls.
* What safety risk exists if the logic is wrong.
* What units are being used, such as milliseconds, volts, PWM range, Celsius, meters, or degrees.

Prefer comments like this:

```cpp
// Tempo máximo, em milissegundos, antes de parar os motores.
// Se o dashboard parar de enviar comandos, isso impede que o robô continue
// se movendo com um comando antigo.
constexpr int COMMAND_TIMEOUT_MS = 300;
```

Avoid useless comments like this:

```cpp
// Define x como 0.
int x = 0;
```

Comment every non-obvious constant.

Comment every hardware-related decision.

Comment every safety-related decision.

Comment functions that affect motors, GPIO, sensors, telemetry, dashboard commands, or robot mode when the behavior, risk, or intention is not obvious from the function name and nearby code.

When adding a new class, include a short comment in Brazilian Portuguese explaining the class responsibility.

When adding a new function, include a short comment in Brazilian Portuguese explaining what it does if the function is not completely obvious from the name.

Do not overcomment obvious code. The goal is clarity, not visual pollution.

---

## Safety Rules

* Motor commands must fail safe. If commands stop arriving, motors must stop.
* Emergency stop must force motor outputs to zero immediately.
* Emergency stop must have priority over dashboard commands, autonomous logic, and manual control.
* New motor/sensor code must centralize pin numbers in `include/obr/config.h`.
* Never hardcode real credentials in scripts or source files. Use SSH keys for deploy.
* Never allow stale dashboard commands to keep controlling the robot forever.
* If sensor data is invalid, missing, or outdated, the robot must choose the safer behavior.
* Startup behavior must be safe: motors should start stopped.
* Shutdown behavior must be safe: motors should be set to zero before exit when possible.
* Any new control mode must clearly define how motors are stopped.

---

## Architecture Rules

* `main.cpp` should only wire modules together.
* Dashboard/websocket code belongs in `DashboardServer`.
* Robot mode and drive values belong in `RobotState`.
* CPU/temperature/RAM readings belong in `Telemetry`.
* GPIO and motor driver details belong in `GpioPin` and `MotorController`.
* Pin numbers and hardware constants belong in `include/obr/config.h`.
* Keep modules small and focused.
* Do not put hardware logic inside dashboard code.
* Do not put dashboard/websocket logic inside motor or GPIO code.
* Do not put business logic directly in `main.cpp`.
* Do not create a new module unless it has a clear responsibility.

Recommended ownership:

```txt
main.cpp
  Wires the application together and runs the main loop.

RobotState
  Stores current robot mode, drive commands, emergency stop state, and command timestamps.

DashboardServer
  Receives commands from the dashboard and sends telemetry back.

Telemetry
  Reads Raspberry Pi CPU usage, temperature, RAM usage, and other system data.

GpioPin
  Low-level GPIO wrapper.

MotorController
  Converts safe drive commands into motor driver outputs.

config.h
  Central place for pins, constants, limits, and hardware configuration.
```

Even though identifiers should stay in English, comments inside these files must be in Brazilian Portuguese.

---

## Simplicity Rules

When adding new code, choose the simplest working version first.

Before adding complexity, ask:

* Can this be done with one small function?
* Can this be configured with a constant in `config.h`?
* Can this be tested without real motors?
* Can a teammate understand this in less than 5 minutes?
* Is this abstraction actually needed now?

Do not add:

* A plugin system.
* A complex event bus.
* A dependency injection framework.
* A large state machine framework.
* A database.
* A complex build system.
* A frontend framework feature unless the dashboard really needs it.

The robot should remain easy to compile, deploy, and debug on the Raspberry Pi.

---

## C++ Rules

* Prefer C++17-compatible code unless the project explicitly changes the standard.
* Prefer clear names over short names.
* Prefer `constexpr` for constants.
* Prefer `enum class` for robot modes and command states.
* Prefer small functions.
* Prefer explicit units in variable names.

Good examples:

```cpp
int commandTimeoutMs;
double cpuTemperatureCelsius;
double ramUsagePercent;
double leftMotorPower;
bool emergencyStopActive;
```

Bad examples:

```cpp
int t;
double temp;
double pwr;
bool flag;
```

* Avoid raw owning pointers.
* Avoid global mutable state unless there is a strong reason.
* Avoid blocking delays in the main control loop.
* Avoid silent failure. Log or expose errors when possible.
* Clamp motor outputs before sending them to hardware.
* Validate dashboard inputs before applying them to the robot.

---

## Dashboard and Command Rules

The dashboard is allowed to send commands, but the robot code must remain in control of safety.

Dashboard commands must not directly control GPIO pins.

Dashboard commands should update `RobotState`.

Robot logic should read from `RobotState` and decide what is safe to apply.

All received dashboard commands should be validated.

If a dashboard command is invalid:

* Ignore it.
* Keep the previous safe state or stop the robot.
* Log the problem if logging exists.

If dashboard connection is lost:

* The robot must stop motors after the configured timeout.
* Telemetry may continue if possible.
* The robot must not continue using old movement commands forever.

---

## Motor Control Rules

Motor control must be defensive.

Every motor output must be clamped to the valid range before being sent to hardware.

Example:

```cpp
// A faixa de saída do motor vai de -1.0 a 1.0.
// Valores fora dessa faixa são inseguros e devem ser limitados antes
// de chegar ao driver de motor.
double safePower = clamp(rawPower, -1.0, 1.0);
```

Motor code must clearly document, in Brazilian Portuguese:

* Which pin controls which motor.
* Which direction is positive.
* What PWM range is expected.
* What happens during emergency stop.
* What happens when command timeout occurs.

Do not duplicate motor pin numbers across files.

Do not hardcode motor limits outside configuration.

---

## GPIO and Hardware Rules

All pin numbers must be defined in:

```txt
include/obr/config.h
```

Do not scatter pin numbers across source files.

Every pin constant should include a comment in Brazilian Portuguese explaining what it connects to.

Example:

```cpp
// Pino GPIO conectado à entrada PWM do motor esquerdo.
// Alterar este valor exige atualizar a fiação ou revisar a PCB/esquemático.
constexpr int LEFT_MOTOR_PWM_PIN = 18;
```

If a pin is active-low, inverted, PWM-capable, or has a special Raspberry Pi function, document it in Brazilian Portuguese.

---

## Telemetry Rules

Telemetry should be useful for debugging, not just decorative.

Telemetry should grow toward including, when available:

* CPU temper
* RAM usage.ature.
* CPU usage.
* Uptime.
* Robot mode.
* Emergency stop state.
* Last command age.
* Motor command values.
* Dashboard connection state.

Telemetry values should include clear names and units.

Example:

```json
  "ramUsagePercent": 47.8
{
  "cpuTemperatureCelsius": 54.2,
  "lastCommandAgeMs": 120,
  "emergencyStopActive": false
}
```

Avoid ambiguous names like:

```json
{
  "temp": 54.2,
  "age": 120,
  "stop": false
}
```

If telemetry code has comments, they must be written in Brazilian Portuguese with correct spelling and accents.

---

## Deploy Rules

* Windows deploy entrypoint: `scripts/deploy.ps1`.
* Linux/macOS deploy entrypoint: `scripts/deploy.sh`.
* The deployed service name is `obr-robot`.
* Default robot host is `obr.local`.
* Default robot user is `raspberry`.
* Default remote directory is `/home/raspberry/OBR2026K`.
* Deploy scripts should be understandable and commented.
* Deploy scripts should fail clearly when SSH, build, or service restart fails.
* Do not hide errors with silent redirects unless there is a clear reason.
* Avoid requiring many manual steps after deploy.

The intended deploy experience is:

```txt
Run one command on the development computer.
The script copies the code to the Raspberry Pi.
The script builds the project.
The script restarts the robot service.
The robot runs the new code.
```

Comments inside deploy scripts may also be written in Brazilian Portuguese when they explain project-specific behavior.

---

## Service Rules

The deployed service name is:

```txt
obr-robot
```

The service should:

* Start the robot application.
* Restart only when appropriate.
* Run with the expected user permissions.
* Log enough information to debug startup failures.
* Stop motors safely when the application exits if possible.

If systemd files are changed, document:

* What service is affected.
* What command is executed.
* What user runs the process.
* Where logs can be checked.

Comments related to the service must clearly explain the effect of each configuration.

---

## Logging and Debugging Rules

Prefer clear logs for important events:

* Robot startup.
* Dashboard connected.
* Dashboard disconnected.
* Emergency stop activated.
* Emergency stop cleared.
* Command timeout.
* Invalid command received.
* Motor outputs forced to zero.
* Sensor read failure.
* Service shutdown.

Logs should be short but useful.

Bad:

```txt
error
```

Good:

```txt
Dashboard command ignored: leftMotorPower was outside [-1.0, 1.0]
```

Do not spam logs inside fast loops unless throttled.

Comments explaining logs must be written in Brazilian Portuguese.

---

## Configuration Rules

Robot constants should be centralized.

Use `include/obr/config.h` for:

* Pin numbers.
* Motor limits.
* Timeout values.
* Dashboard port.
* Telemetry interval.
* Safety thresholds.
* Hardware-specific constants.

Every important constant should have a comment in Brazilian Portuguese explaining:

* Unit.
* Purpose.
* What happens if it is too high or too low.

Example:

```cpp
// Potência máxima permitida para os motores.
// Reduza este valor durante os primeiros testes para diminuir a velocidade
// do robô e reduzir o risco de colisões.
constexpr double MAX_MOTOR_OUTPUT = 0.40;
```

---

## Testing Rules

When changing robot behavior, prefer testing in this order:

1. Build locally.
2. Run without motors connected if possible.
3. Run with motors lifted off the ground.
4. Run at reduced motor power.
5. Test on the floor.
6. Test under competition-like conditions.

New code should not assume motors are connected.

Dashboard and telemetry code should be testable without the full robot hardware when possible.

If adding behavior that affects movement, include a clear manual test checklist in the response or pull request.

---

## Pull Request / Change Rules

When making changes, explain:

* What changed.
* Why it changed.
* What files were affected.
* How to test it.
* What safety behavior was preserved or added.
* Any assumptions made.

For robot-control changes, include:

```txt
Safety check:
- Motors stop on emergency stop: yes/no
- Motors stop on command timeout: yes/no
- Motor output clamped: yes/no
- Pins centralized in config.h: yes/no
```

When adding comments, verify:

```txt
Comment quality check:
- Comments are in Brazilian Portuguese: yes/no
- Spelling and accents were reviewed: yes/no
- Comments explain purpose, effect, or safety risk: yes/no
- Comments avoid obvious noise: yes/no
```

---

## Documentation Rules

Keep documentation practical.

Documentation should help a teammate:

* Build the project.
* Deploy to the Raspberry Pi.
* Understand the architecture.
* Debug common errors.
* Know where to change pins and constants.
* Know how safety logic works.

Avoid documentation that sounds impressive but does not help operate the robot.

---

## Forbidden Patterns

Do not:

* Put real passwords, Wi-Fi credentials, tokens, or private keys in the repo.
* Hardcode Raspberry Pi credentials in source code.
* Scatter GPIO pin numbers across multiple files.
* Send motor commands without clamping.
* Let old dashboard commands control the robot forever.
* Put all logic in `main.cpp`.
* Add a dependency without explaining why.
* Hide errors during deploy.
* Make the dashboard bypass safety logic.
* Assume the robot is safe just because the dashboard says stop.
* Write comments in broken Portuguese.
* Write Portuguese comments without required accents.
* Add vague comments that do not explain purpose, effect, or risk.

---

## Preferred Agent Behavior

When an AI coding agent modifies this repository, it should:

* Keep changes small and focused.
* Explain the reasoning behind the change.
* Prefer simple C++ over clever C++.
* Add clear comments in Brazilian Portuguese to important code.
* Review spelling, accents, and grammar in comments before finishing.
* Preserve the deploy flow.
* Preserve safety behavior.
* Avoid unnecessary dependencies.
* Ask before introducing a major library or architecture change.
* Make the code readable for a robotics team, not just for senior software engineers.
* Include exact commands to build, run, or test when relevant.
* Mention any file that needs to be edited manually.

The agent should not rewrite the whole project unless explicitly requested.

The agent should not introduce a complex architecture when a simple module is enough.

The agent should not optimize prematurely.

The agent should not remove comments that explain hardware, safety, or architecture decisions.

---

## Final Principle

This is a competition robot project.

A simple system that the team understands is better than an advanced system that only one person can debug.

Safety, clarity, reliable deploy, and well-explained Portuguese comments matter more than elegant code.

The key point is: identifiers stay in English, while source comments that explain behavior, safety, or hardware decisions use correct Brazilian Portuguese.
