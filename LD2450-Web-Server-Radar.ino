/*
Created by donk3ybrain - 2025 - https://github.com/donk3ybrain/LD2450-Web-Server-Radar

Arduino IDE 1.8.19
ESP32 v3.0.7 Board Files
For ESP32-S3 with 5V Out (tested on WeAct S3 Mini)

ESP32-S3  to  HLK-LD2450:
G         ->   G
5v        ->   5v
17        ->   TX
16        ->   RX
*/

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

//── Serial1 configuration ──────────────────────────────────────────────────────
#define SERIAL1_RX_PIN  17
#define SERIAL1_TX_PIN  16
#define BAUD_RATE       256000

//── Wifi Config ──────────────────────────────────────────────────────
const char* ssid     = "myRadar";
const char* password = "myPassword!";
IPAddress local_ip(10,9,8,1);
IPAddress gateway(10,9,8,1);
IPAddress subnet(255,255,255,0);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//── Frame & buffer parameters ─────────────────────────────────────────────────
static const int BUF_SIZE     = 256;
static const int FRAME_SIZE   = 30;
static const int ZERO_TOL     = 3;         // allowed zero-only frames
static const TickType_t STATUS_INTERVAL = pdMS_TO_TICKS(10000);

//── Ring buffer & synchronization ─────────────────────────────────────────────
static uint8_t ringBuf[BUF_SIZE];
static int     bufLen     = 0;
static portMUX_TYPE bufMux = portMUX_INITIALIZER_UNLOCKED;

//── Frame markers ─────────────────────────────────────────────────────────────
static const uint8_t HDR[4] = { 0xAA, 0xFF, 0x03, 0x00 };
static const uint8_t FTR[2] = { 0x55, 0xCC };

//── Health‐monitoring counters ────────────────────────────────────────────────
static volatile uint32_t droppedBytes     = 0;
static volatile uint32_t malformedFrames  = 0;
static volatile uint32_t activationResets = 0;
static volatile uint32_t wildJumpClamps   = 0;

//── Task handles for stack‐watermark checks ───────────────────────────────────
static TaskHandle_t readTaskHandle, parseTaskHandle, statusTaskHandle;

//── Target‐tracking state ─────────────────────────────────────────────────────
struct TargetState {
  int16_t  last_x     = 0;
  int16_t  last_y     = 0;
  int16_t  last_speed = 0;
  uint16_t last_dist  = 0;
  int      goodCount  = 0;
  int      zeroCount  = 0;
  bool     active     = false;
} targetStates[3];

//── Raw‐frame & decoded‐target buffers ────────────────────────────────────────
static uint8_t frameBuf[FRAME_SIZE];
static uint8_t localBuf[BUF_SIZE];   // snapshot buffer

struct RawTarget {
  int16_t  x;
  int16_t  y;
  int16_t  speed;
  uint16_t dist;
};

//── Forward declarations ──────────────────────────────────────────────────────
void readSerialTask(void*);
void parseFramesTask(void*);
void statusTask(void*);
void processFrame(const uint8_t *);

//── WebSocket Event handling ───────────────────────────────────────────────────
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

//── HTML page ──────────────────────────────────────────────────────
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Radar</title>
  <meta name="viewport" content="width=device-width,height=device-height,initial-scale=1.0">
  <script>
    function lockToLandscape() {
      const s      = screen;
      const locker = s.orientation?.lock
                  || s.lockOrientation
                  || s.mozLockOrientation
                  || s.msLockOrientation;
      if (locker) {
        locker.call(s, 'landscape')
              .catch(e => console.warn('Lock failed', e));
      }
    }
    window.addEventListener('load', lockToLandscape);
    window.addEventListener('orientationchange', lockToLandscape);
  </script>

  <style>
    body {
      background: #222;
      margin: 0; overflow: hidden;
    }
    @media screen and (orientation: portrait) {
      body {
        transform: rotate(90deg);
        transform-origin: top left;
        width: 100vh; height: 100vw;
        position: absolute; top: 0; left: 0;
      }
    }
    canvas { display: block; margin: auto; }
    button {
      outline: 0;
      border: 1px solid transparent;
      background-color: rgba(10,10,10,0);
      color: rgba(3,194,252,0.95);
      position: absolute; top: 20px; right: 20px;
      z-index: 10; padding: 4px 8px;
      font-size: 38px;
    transform: rotate(90deg);
    transition: transform 1s;
    }
  </style>
</head>
<body>
  <button id="rotateBtn">&#8631;</button>
  <span>&nbsp<br></span>
  <canvas id="radar" width="620" height="320"></canvas>

  <script>
  // ————— Setup & State —————
  const canvas    = document.getElementById('radar');
  const ctx       = canvas.getContext('2d');
  const offscreen = new OffscreenCanvas(canvas.width, canvas.height);
  const offctx    = offscreen.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cX = W/2, cY = H;
  const rotateBtn = document.getElementById('rotateBtn');

  let rotated = false;

  rotateBtn.addEventListener('click', () => {
    rotated = !rotated;
    renderFrame(1);
    const baseAngle = 90;  // the initial CSS rule
    const extra     = rotated ? 180 : 0;
    rotateBtn.style.transform = `rotate(${baseAngle + extra}deg)`;
  });

  const targets = [
    { t:1,x:0,y:0,prevX:0,prevY:0 },
    { t:2,x:0,y:0,prevX:0,prevY:0 },
    { t:3,x:0,y:0,prevX:0,prevY:0 }
  ];
  const active = [false,false,false];

  function lerp(a,b,t){ return a + (b - a)*t; }

  // ————— Draw static grid shapes (no labels) —————
  function drawGridShapes() {
    offctx.clearRect(0,0,W,H);
    offctx.strokeStyle = 'rgba(0,255,38,0.5)';
    offctx.lineWidth   = 1.3;

    // arcs
    for (let r=50, lbl=1; r<=300; r+=50, lbl++) {
      offctx.beginPath();
      offctx.arc(cX, cY, r,
                 198*Math.PI/180,
                 342*Math.PI/180);
      offctx.stroke();
    }
    // radials
    for (let a=198*Math.PI/180; a<=342*Math.PI/180; a+=Math.PI/10) {
      offctx.beginPath();
      offctx.moveTo(cX,cY);
      offctx.lineTo(
        cX + Math.cos(a)*300,
        cY + Math.sin(a)*300
      );
      offctx.stroke();
    }
  }

  // ————— Draw arc-labels always upright —————
  function drawArcLabels() {
    ctx.fillStyle    = 'rgba(3,194,252,0.95)';
    ctx.font         = '11px Arial';
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';

    const angle = 198*Math.PI/180;
    for (let r=50, lbl=1; r<=300; r+=50, lbl++) {
      // original label point:
      let x = cX + Math.cos(angle)*(r + 6);
      let y = cY + Math.sin(angle)*(r - 34);

      // if flipped, mirror about bottom-right:
      if (rotated) {
        x = W - x;
        y = H - y;
      }
      ctx.fillText(`${lbl}M`, x, y);
    }
  }

  // ————— Draw a single target dot —————
  function drawTarget(tar) {
    const color = tar.t===1 ? 'orangered'
                : tar.t===2 ? 'yellow'
                :               'deeppink';

    const rad  = (tar.y/6000)*300;
    const xoff = (tar.x/500)*rad;
    const px   = cX + xoff;
    const py   = cY - rad;

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(px,py,8,0,2*Math.PI);
    ctx.fill();

    ctx.fillStyle = 'white';
    ctx.font      = '16px Arial';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';

    if (rotated) {
      ctx.save();
      ctx.translate(px + 10, py);
      ctx.rotate(Math.PI);
      ctx.fillText(`${tar.t}`, 20, 0);
      ctx.restore();
    } else {
      ctx.fillText(`${tar.t}`, px + 10, py);
    }
  }

  // ————— Render one frame at given progress [0–1] —————
  function renderFrame(progress) {
    ctx.clearRect(0,0,W,H);

    // 1) draw grid+targets (flipped if needed)
    ctx.save();
    if (rotated) {
      ctx.translate(W,H);
      ctx.rotate(Math.PI);
    }
    ctx.drawImage(offscreen, 0, 0);
    targets.forEach((t,i) => {
      if (active[i]) {
        const ix = lerp(t.prevX, t.x, progress);
        const iy = lerp(t.prevY, t.y, progress);
        drawTarget({t:t.t, x:ix, y:iy});
      }
    });
    ctx.restore();

    drawArcLabels();
  }

  // ————— WebSocket + Animation —————
  const ws = new WebSocket('ws://10.9.8.1/ws');
  ws.onmessage = e => {
    const {t,x,y} = JSON.parse(e.data);
    const i       = t - 1;
    targets[i].prevX = targets[i].x;
    targets[i].prevY = targets[i].y;
    targets[i].x     = x;
    targets[i].y     = y;
    active[i]        = true;

    const start = performance.now();
    const dur   = 100;
    function step(ts) {
      const p = Math.min((ts - start)/dur, 1);
      renderFrame(p);
      if (p < 1) requestAnimationFrame(step);
    }
    requestAnimationFrame(step);
  };

  // ————— Kickoff —————
  drawGridShapes();
  renderFrame(1);

  </script>
</body>
</html>
)rawliteral";

//── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);

  delay(100);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  
  delay(200);
  
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.begin();
  delay(200);

  xTaskCreatePinnedToCore(
    readSerialTask, "ReadSerial", 2048, NULL, 3, &readTaskHandle, 1
  );
  xTaskCreatePinnedToCore(
    parseFramesTask, "ParseFrames", 4096, NULL, 2, &parseTaskHandle, 0
  );
  xTaskCreate(
    statusTask, "Status", 2048, NULL, 1, &statusTaskHandle
  );
}

//── Loop does nothing; tasks handle all work ──────────────────────────────────
void loop() {
  ws.cleanupClients();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

//── Task: Read one byte at a time under lock ──────────────────────────────────
void readSerialTask(void *pv) {
  (void)pv;
  for (;;) {
    if (Serial1.available()) {
      uint8_t b = Serial1.read();
      portENTER_CRITICAL(&bufMux);
        if (bufLen < BUF_SIZE) {
          ringBuf[bufLen++] = b;
        } else {
          droppedBytes++;
        }
      portEXIT_CRITICAL(&bufMux);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

//── Task: Snapshot buffer, scan all frames, then remove them ─────────────────
void parseFramesTask(void *pv) {
  (void)pv;
  for (;;) {
    int snapshotLen;

    // 1) COPY under lock
    portENTER_CRITICAL(&bufMux);
      snapshotLen = bufLen;
      memcpy(localBuf, ringBuf, snapshotLen);
    portEXIT_CRITICAL(&bufMux);

    // 2) SCAN snapshot
    int consumed = 0;
    while (snapshotLen - consumed >= FRAME_SIZE) {
      if (memcmp(&localBuf[consumed], HDR, 4) == 0) {
        memcpy(frameBuf, &localBuf[consumed], FRAME_SIZE);
        if (frameBuf[28] == FTR[0] && frameBuf[29] == FTR[1]) {
          processFrame(frameBuf);
          consumed += FRAME_SIZE;
        } else {
          malformedFrames++;
          consumed++;
        }
      } else {
        consumed++;
      }
    }

    // 3) REMOVE under lock
    if (consumed > 0) {
      portENTER_CRITICAL(&bufMux);
        if (consumed <= bufLen) {
          memmove(ringBuf, ringBuf + consumed, bufLen - consumed);
          bufLen -= consumed;
        }
      portEXIT_CRITICAL(&bufMux);
    }

    // fast-path: more work?
    if (bufLen >= FRAME_SIZE) {
      taskYIELD();
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

//── Frame‐processing & validation logic ──────────────────────────────────────
void processFrame(const uint8_t *f) {
  RawTarget raw[3];

  // helper: custom sign-bit decode
  auto decodeSI = [](uint16_t v) {
    int16_t mag = v & 0x7FFF;
    bool    pos = v & 0x8000;
    return pos ? mag : -mag;
  };

  // decode each target’s 8-byte block
  for (int i = 0; i < 3; i++) {
    int b = 4 + i * 8;
    uint16_t ux = f[b]     | (f[b+1] << 8);
    uint16_t uy = f[b+2]   | (f[b+3] << 8);
    uint16_t us = f[b+4]   | (f[b+5] << 8);
    uint16_t ud = f[b+6]   | (f[b+7] << 8);

    raw[i].x     = decodeSI(ux);
    raw[i].y     = decodeSI(uy);
    raw[i].speed = decodeSI(us);
    raw[i].dist  = ud;
  }

  // validate & update state
  for (int i = 0; i < 3; i++) {
    auto &st = targetStates[i];
    auto  rt = raw[i];

    // 1) no-data check
    if (rt.x==0 && rt.y==0 && rt.speed==0 && rt.dist==0) {
      st.zeroCount++;
      st.goodCount = 0;
      if (st.zeroCount > ZERO_TOL) {
        st.active = false;
        activationResets++;
      }
      continue;
    }
    st.zeroCount = 0;

    // 2) activation build-up
    if (rt.y > 0) {
      if (++st.goodCount >= 3) st.active = true;
    } else {
      st.goodCount = 0;
    }

    // 3) clamp wild jumps if active
    if (st.active) {
      int dx = int(rt.x) - int(st.last_x);
      int dy = int(rt.y) - int(st.last_y);
      if (abs(dx) >= 300) { rt.x = st.last_x; wildJumpClamps++; }
      if (abs(dy) >= 300) { rt.y = st.last_y; wildJumpClamps++; }
    }

    // 4) save last values
    st.last_x     = rt.x;
    st.last_y     = rt.y;
    st.last_speed = rt.speed;
    st.last_dist  = rt.dist;
  }

  // 5) print only active targets
for (int i = 0; i < 3; i++) {
  auto &st = targetStates[i];
  if (!st.active) continue;

  // build the message
  String message = String("{\"t\":") + String(i + 1)
                 + ",\"x\":" + String(st.last_x)
                 + ",\"y\":" + String(st.last_y) + "}";
  Serial.println(message);
  ws.textAll(message);
}
}

//── Task: Health & stack-usage dump ───────────────────────────────────────────
void statusTask(void *pv) {
  (void)pv;
  TickType_t nextWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&nextWake, STATUS_INTERVAL);

    UBaseType_t readHi  = uxTaskGetStackHighWaterMark(readTaskHandle);
    UBaseType_t parseHi = uxTaskGetStackHighWaterMark(parseTaskHandle);
    UBaseType_t statHi  = uxTaskGetStackHighWaterMark(statusTaskHandle);

    Serial.printf(
      "[Status] Drop:%u  Mal:%u  Resets:%u  Clamps:%u  "
      "StackHi(Read:%u Parse:%u Stat:%u)\n",
      droppedBytes, malformedFrames, activationResets, wildJumpClamps,
      readHi, parseHi, statHi
    );
  }
}


