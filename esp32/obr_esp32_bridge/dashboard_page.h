#pragma once

#include <Arduino.h>

const char DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pt-BR">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>OBR2026K | ESP32</title>
  <style>
    :root{color-scheme:dark;font-family:Inter,system-ui,sans-serif;background:#08111c;color:#e8f0f7}
    *{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at top,#12283a,#08111c 52%)}
    main{max-width:1100px;margin:auto;padding:18px}.top{display:flex;gap:16px;align-items:center;justify-content:space-between;flex-wrap:wrap}
    h1{margin:0;font-size:clamp(24px,5vw,38px)}.sub,.muted{color:#93a8b8}.badge{padding:8px 12px;border-radius:999px;background:#392126;color:#ff9b9b;font-weight:800}.badge.ok{background:#113a2b;color:#70e6ad}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;margin:18px 0}.card{background:#101e2b;border:1px solid #21384a;border-radius:14px;padding:15px;box-shadow:0 10px 28px #0004}
    .card h2{font-size:14px;text-transform:uppercase;letter-spacing:.09em;color:#8aa5b8;margin:0 0 12px}.value{font-size:30px;font-weight:850;font-variant-numeric:tabular-nums}.unit{font-size:14px;color:#8aa5b8}
    .line{display:flex;justify-content:space-between;gap:12px;padding:5px 0}.line span:last-child{font-variant-numeric:tabular-nums;text-align:right}
    .controls{display:grid;grid-template-columns:1fr 1fr;gap:12px}.wide{grid-column:1/-1}label{display:flex;justify-content:space-between;margin:8px 0}input[type=range]{width:100%;accent-color:#36b9e6}
    button{border:0;border-radius:10px;min-height:46px;padding:10px 14px;background:#176d91;color:white;font-weight:800;font-size:15px;cursor:pointer}button.secondary{background:#344b5e}button.danger{background:#ba283b}button:disabled{opacity:.45;cursor:not-allowed}
    .drive{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:420px;margin:auto}.drive .forward{grid-column:2}.drive .left{grid-column:1}.drive .stop{grid-column:2;background:#ba283b}.drive .right{grid-column:3}.drive .reverse{grid-column:2}
    .keyboard-panel{display:flex;align-items:center;justify-content:center;gap:22px;padding:12px;border:1px solid #29475d;border-radius:12px;background:#0b1823}.keyboard-copy{max-width:330px}.keyboard-copy strong{display:block;margin-bottom:5px}.keys{display:grid;grid-template-columns:repeat(3,42px);grid-template-rows:repeat(2,42px);gap:6px}.keycap{display:grid;place-items:center;border:1px solid #3c6078;border-bottom-width:4px;border-radius:8px;background:#172c3b;color:#b9cfdd;font-weight:900;transition:.08s}.keycap.w{grid-column:2}.keycap.a{grid-column:1}.keycap.s{grid-column:2}.keycap.d{grid-column:3}.keycap.active{transform:translateY(2px);border-bottom-width:2px;background:#1d88b3;color:white;box-shadow:0 0 16px #36b9e655}
    table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;border-bottom:1px solid #21384a;padding:7px 5px}th{color:#8aa5b8}code{color:#70d7ff}
    .warn{border-left:4px solid #f2b84b;padding:10px 12px;background:#332a16;border-radius:6px}.error{color:#ff8795}.good{color:#70e6ad}.recovering{color:#f2b84b}@media(max-width:600px){.controls{grid-template-columns:1fr}.wide{grid-column:auto}.keyboard-panel{flex-direction:column;text-align:center}}
  </style>
</head>
<body>
<main>
  <div class="top"><div><h1>OBR2026K · ESP32</h1><div class="sub">Firmware v20 · perfis de reta e giro · controle WASD</div></div><div id="connection" class="badge">Conectando…</div></div>
  <div class="grid">
    <section class="card"><h2>Bateria 12 V</h2><div><span id="battery" class="value">--</span> <span class="unit">V</span></div><div class="line"><span>ADC GPIO36</span><span id="batteryAdc">-- mV</span></div></section>
    <section class="card"><h2>Ultrassônico frontal</h2><div><span id="distance" class="value">--</span> <span class="unit">cm</span></div><div id="ultrasonicState" class="muted">Sem leitura</div></section>
    <section class="card"><h2>Motores</h2><div class="line"><span>Lado esquerdo (2 motores)</span><span id="leftApplied">0.00</span></div><div class="line"><span>Lado direito (2 motores)</span><span id="rightApplied">0.00</span></div><div class="line"><span>Conversão</span><span class="good">PWM direto 1:1</span></div><div class="line"><span>GPIO26 nSLEEP</span><span id="driverState">inicializando</span></div><div class="line"><span>Fonte</span><span id="source">nenhuma</span></div><div class="line"><span>Último comando</span><span id="commandAge">-- ms</span></div></section>
    <section class="card"><h2>Encoders</h2><div class="line"><span>Esquerdo</span><span id="leftEncoder">0</span></div><div class="line"><span>Velocidade E</span><span id="leftRate">0 cont/s</span></div><div class="line"><span>Direito</span><span id="rightEncoder">0</span></div><div class="line"><span>Velocidade D</span><span id="rightRate">0 cont/s</span></div><button class="secondary" onclick="post('/api/reset-encoders')">Zerar contadores</button></section>
    <section class="card"><h2>MPU6050</h2><div class="line"><span>Estado</span><span id="mpuState">--</span></div><div class="line"><span>Aceleração X/Y/Z</span><span id="accel">--</span></div><div class="line"><span>Giro X/Y/Z</span><span id="gyro">--</span></div><div class="line"><span>Giro integrado</span><span id="yaw">--°</span></div><div class="line"><span>Inclinação da rampa</span><span id="rampAngle">--°</span></div><div class="line"><span>Temperatura</span><span id="imuTemp">-- °C</span></div></section>
    <section class="card"><h2>I2C e sistema</h2><div class="line"><span>MPU6050</span><span id="mpuAddress">--</span></div><div class="line"><span>PCA9685</span><span id="pcaState">--</span></div><div class="line"><span>SSD1306</span><span id="oledState">--</span></div><div class="line"><span>Botão GPIO27</span><span id="buttonState">--</span></div><div class="line"><span>Clientes Wi‑Fi</span><span id="clients">0</span></div><div class="line"><span>Uptime</span><span id="uptime">--</span></div></section>
  </div>

  <section class="card">
    <h2>Controle de bancada</h2>
    <p class="warn">Levante as rodas antes do primeiro teste. O painel desarma se deixar de enviar comandos por 500 ms.</p>
    <p class="muted">Controle direto: 5% no slider produz 5% de PWM. Cada lado é independente, sem mínimo, remapeamento ou boost automático.</p>
    <div class="controls">
      <div class="keyboard-panel wide">
        <div class="keyboard-copy"><strong>Controle pelo teclado</strong><span id="keyboardState" class="muted">Habilite os motores para usar WASD.</span></div>
        <div class="keys" aria-label="Teclas de movimento"><span id="keyW" class="keycap w">W</span><span id="keyA" class="keycap a">A</span><span id="keyS" class="keycap s">S</span><span id="keyD" class="keycap d">D</span></div>
      </div>
      <div><label>Lado esquerdo · 2 motores <output id="leftValue">0.00</output></label><input id="left" type="range" min="-100" max="100" value="0"></div>
      <div><label>Lado direito · 2 motores <output id="rightValue">0.00</output></label><input id="right" type="range" min="-100" max="100" value="0"></div>
      <button id="arm" onclick="armDashboard()">Habilitar motores</button><button class="secondary" onclick="disarmDashboard()">Desabilitar</button>
      <button class="danger wide" onclick="emergencyStop()">PARADA DE EMERGÊNCIA</button>
      <button id="clearEstop" class="secondary wide" onclick="clearEmergencyStop()">Liberar E‑Stop (mantém parado)</button>
      <div class="drive wide">
        <button class="forward" data-l="1.00" data-r="1.00">Frente 100%</button><button class="left" data-l="-1.00" data-r="1.00">Esquerda 100%</button><button class="stop" data-l="0" data-r="0">Parar</button><button class="right" data-l="1.00" data-r="-1.00">Direita 100%</button><button class="reverse" data-l="-1.00" data-r="-1.00">Ré 100%</button>
      </div>
    </div>
  </section>

  <section class="card"><h2>Mapa de pinos</h2><table><thead><tr><th>GPIO</th><th>Função</th><th>GPIO</th><th>Função</th></tr></thead><tbody>
    <tr><td><code>32 / 33</code></td><td>Ultrassônico TRIG / ECHO</td><td><code>27</code></td><td>Start button</td></tr><tr><td><code>14 / 13</code></td><td>I2C SCL / SDA: MPU, PCA e OLED</td><td><code>26</code></td><td>DRV8833 nSLEEP</td></tr><tr><td><code>5 / 18</code></td><td>DRV8833 esquerdo IN1 / IN2</td><td><code>16 / 17</code></td><td>DRV8833 direito IN1 / IN2</td></tr><tr><td><code>22 / 23</code></td><td>Encoder esquerdo A / B</td><td><code>19 / 21</code></td><td>Encoder direito A / B</td></tr><tr><td><code>1 / 3</code></td><td>UART TX / RX Raspberry</td><td><code>36</code></td><td>Bateria, divisor 47 kΩ / 10 kΩ</td></tr>
  </tbody></table></section>
</main>
<script>
  const $=id=>document.getElementById(id), left=$('left'), right=$('right'); let armed=false, motorRequestInFlight=false, motorCommandPending=false;
  const driveKeyCodes=['KeyW','KeyA','KeyS','KeyD'], pressedDriveKeys=new Set();
  const n=(value,digits=2)=>Number(value).toFixed(digits);
  async function post(path,params={}){const query=new URLSearchParams(params).toString();try{const response=await fetch(path+(query?'?'+query:''),{method:'POST',cache:'no-store'});const data=await response.json();if(!response.ok)throw new Error(data.error||'falha');return data}catch(error){$('connection').textContent=error.message;$('connection').className='badge';throw error}}
  function setDrive(l,r){left.value=Math.round(l*100);right.value=Math.round(r*100);updateLabels();if(armed)sendMotor()}
  function updateLabels(){$('leftValue').textContent=n(Number(left.value)/100);$('rightValue').textContent=n(Number(right.value)/100)}
  function updateKeyboardIndicators(){driveKeyCodes.forEach(code=>$(code.replace('Key','key')).classList.toggle('active',pressedDriveKeys.has(code)));$('keyboardState').textContent=!armed?'Habilite os motores para usar WASD.':(pressedDriveKeys.size?'Teclado comandando os dois lados.':'W/S: frente e ré · A/D: giro com os dois lados.')}
  function resetKeyboardState(){pressedDriveKeys.clear();updateKeyboardIndicators()}
  function applyKeyboardDrive(){const forward=(pressedDriveKeys.has('KeyW')?1:0)-(pressedDriveKeys.has('KeyS')?1:0),turn=(pressedDriveKeys.has('KeyD')?1:0)-(pressedDriveKeys.has('KeyA')?1:0);if(turn<0)setDrive(-1,1);else if(turn>0)setDrive(1,-1);else setDrive(forward,forward)}
  function stopDriveOnFocusLoss(){resetKeyboardState();if(armed)setDrive(0,0)}
  async function sendMotor(){if(!armed)return;motorCommandPending=true;if(motorRequestInFlight)return;motorRequestInFlight=true;try{while(armed&&motorCommandPending){motorCommandPending=false;await post('/api/motor',{left:Number(left.value)/100,right:Number(right.value)/100})}}catch(e){armed=false;motorCommandPending=false;resetKeyboardState();left.value=0;right.value=0;updateLabels()}finally{motorRequestInFlight=false}}
  async function armDashboard(){resetKeyboardState();setDrive(0,0);try{await post('/api/arm');armed=true;motorCommandPending=false;updateKeyboardIndicators()}catch(e){armed=false;updateKeyboardIndicators()}}
  async function disarmDashboard(){armed=false;motorCommandPending=false;resetKeyboardState();setDrive(0,0);await post('/api/disarm')}
  async function emergencyStop(){armed=false;motorCommandPending=false;resetKeyboardState();setDrive(0,0);await post('/api/estop')}
  async function clearEmergencyStop(){armed=false;motorCommandPending=false;resetKeyboardState();setDrive(0,0);await post('/api/clear-estop')}
  async function refresh(){try{const r=await fetch('/api/telemetry',{cache:'no-store'}),d=await r.json();$('connection').textContent=d.emergencyStop?'E‑STOP ATIVO':(d.dashboardArmed?'CONTROLE HABILITADO':'CONECTADO · PARADO');$('connection').className='badge '+(!d.emergencyStop?'ok':'');armed=d.dashboardArmed&&!d.emergencyStop;if(!armed&&pressedDriveKeys.size)resetKeyboardState();
    $('battery').textContent=n(d.batteryVoltage);$('batteryAdc').textContent=n(d.batteryAdcMillivolts,0)+' mV';$('distance').textContent=d.ultrasonicValid?n(d.ultrasonicDistanceCm,1):'--';$('ultrasonicState').textContent=d.ultrasonicValid?'Leitura válida':'Sem eco válido';
    $('leftApplied').textContent=n(d.leftMotorPower);$('rightApplied').textContent=n(d.rightMotorPower);$('driverState').textContent=d.motorSleepPinHigh?'HIGH · habilitado':'LOW · verificar';$('driverState').className=d.motorSleepPinHigh?'good':'error';$('source').textContent=d.controlSource;$('commandAge').textContent=d.lastCommandAgeMs+' ms';$('leftEncoder').textContent=d.leftEncoderCount;$('rightEncoder').textContent=d.rightEncoderCount;$('leftRate').textContent=n(d.leftEncoderRate,0)+' cont/s';$('rightRate').textContent=n(d.rightEncoderRate,0)+' cont/s';
    $('mpuState').textContent=d.mpuReady?'online':'indisponível';$('mpuState').className=d.mpuReady?'good':'error';$('accel').textContent=`${n(d.accelX)} / ${n(d.accelY)} / ${n(d.accelZ)} m/s²`;$('gyro').textContent=`${n(d.gyroX)} / ${n(d.gyroY)} / ${n(d.gyroZ)} °/s`;$('yaw').textContent=n(d.yawZ,1)+'°';$('rampAngle').textContent=n(d.rampAngleDegrees,1)+'°';$('imuTemp').textContent=n(d.imuTemperatureCelsius,1)+' °C';$('mpuAddress').textContent=d.mpuReady?'0x'+d.mpuAddress.toString(16).toUpperCase():'não encontrado';$('pcaState').textContent=d.pca9685Ready?'online · 0x40':'indisponível';$('pcaState').className=d.pca9685Ready?'good':'error';$('oledState').textContent=d.oledReady?'online · 0x'+d.oledAddress.toString(16).toUpperCase():'indisponível';$('oledState').className=d.oledReady?'good':'error';$('buttonState').textContent=d.startButtonPressed?'PRESSIONADO':'solto';$('clients').textContent=d.wifiClients;$('uptime').textContent=Math.floor(d.uptimeMs/1000)+' s';$('arm').disabled=d.emergencyStop;$('clearEstop').disabled=!d.emergencyStop;
    updateKeyboardIndicators();
  }catch(e){$('connection').textContent='SEM RESPOSTA';$('connection').className='badge';armed=false;resetKeyboardState();left.value=0;right.value=0;updateLabels()}}
  left.addEventListener('input',()=>setDrive(Number(left.value)/100,Number(right.value)/100));right.addEventListener('input',()=>setDrive(Number(left.value)/100,Number(right.value)/100));document.querySelectorAll('[data-l]').forEach(b=>b.addEventListener('click',()=>setDrive(Number(b.dataset.l),Number(b.dataset.r))));
  document.addEventListener('keydown',event=>{if(!driveKeyCodes.includes(event.code)||event.ctrlKey||event.altKey||event.metaKey)return;event.preventDefault();if(!armed){updateKeyboardIndicators();return}if(!pressedDriveKeys.has(event.code)){pressedDriveKeys.add(event.code);updateKeyboardIndicators();applyKeyboardDrive()}});
  document.addEventListener('keyup',event=>{if(!driveKeyCodes.includes(event.code))return;event.preventDefault();if(pressedDriveKeys.delete(event.code)){updateKeyboardIndicators();if(armed)applyKeyboardDrive()}});
  window.addEventListener('blur',stopDriveOnFocusLoss);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopDriveOnFocusLoss()});window.addEventListener('pagehide',()=>navigator.sendBeacon('/api/disarm'));setInterval(()=>{if(armed)sendMotor()},100);setInterval(refresh,250);updateLabels();updateKeyboardIndicators();refresh();
</script>
</body></html>
)HTML";
