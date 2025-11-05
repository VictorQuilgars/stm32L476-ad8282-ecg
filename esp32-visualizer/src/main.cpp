#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ---------- WiFi AP ----------
const char* AP_SSID = "ECG-ESP32";
const char* AP_PASS = "12345678";

// ---------- Web ----------
WebServer http(80);
WebSocketsServer ws(81);

// Page HTML embarquée
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en"><head><meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>ECG Live</title>
<style>
  body{margin:0;font:14px system-ui,sans-serif;background:#0b1220;color:#e7eefc}
  header{padding:10px 14px;background:#0f1b36;position:sticky;top:0}
  .row{display:flex;gap:16px;align-items:center;flex-wrap:wrap}
  .tag{background:#13254a;padding:6px 10px;border-radius:10px}
  #status{color:#7fd1ff}
  canvas{display:block;width:100vw;height:50vh;background:#071026}
  .info{padding:10px 14px;color:#9bb4d0}
</style>
</head>
<body>
<header class="row">
  <div class="tag">Wi-Fi: <span id="ip">—</span></div>
  <div class="tag">WS: <span id="status">connecting…</span></div>
  <div class="tag">BPM: <span id="bpm">—</span></div>
  <div class="tag">Fs: <span id="fs">~250 sps</span></div>
</header>
<canvas id="ecg"></canvas>
<div class="info">Streaming "FILT=&lt;mV&gt;" and "BPM=&lt;val&gt;" lines.</div>

<script>
const ipEl=document.getElementById('ip');
fetch('/ip').then(r=>r.text()).then(t=>ipEl.textContent=t).catch(()=>{});

const canvas=document.getElementById('ecg');
const ctx=canvas.getContext('2d');
let W=0,H=0; function resize(){ W=canvas.width=innerWidth; H=canvas.height=Math.floor(innerHeight*0.5); }
addEventListener('resize',resize); resize();

// Ring buffer (plus grand que l'écran)
const N=3000; const data=new Float32Array(N); let head=0,size=0;
function push(v){ data[head]=v; head=(head+1)%N; size=Math.min(size+1,N); }

// Baseline & scale (centrage + min/max lissés sur la fenêtre visible)
let dc=0.0;
const ALPHA_DC=0.002;        // baseline très lente
let yMin=-500, yMax=+500;    // bornes lissées entre frames
const ALPHA_MM=0.25;         // lissage min/max (0..1)
let lastRedraw=0;

// map valeur -> pixel Y avec yMin/yMax lissés
function yOf(v){
  const t=(v - yMin) / Math.max(1e-9, (yMax - yMin));
  return H - Math.max(0, Math.min(1, t)) * H;
}

function draw(ts){
  if(ts-lastRedraw<30){ requestAnimationFrame(draw); return; } // ~33 fps
  lastRedraw=ts;
  ctx.clearRect(0,0,W,H);

  // grille
  ctx.globalAlpha=0.15; ctx.strokeStyle='#4b5f86'; ctx.lineWidth=1; ctx.beginPath();
  for(let x=0;x<W;x+=W/10){ ctx.moveTo(x,0); ctx.lineTo(x,H); }
  for(let y=0;y<H;y+=H/8){ ctx.moveTo(0,y); ctx.lineTo(W,y); }
  ctx.stroke(); ctx.globalAlpha=1;

  if(size>1){
    const marginRight=Math.max(20,Math.floor(W*0.12));   // zone “temps réel” à droite
    const drawW=Math.max(1,W-marginRight);               // largeur utile
    const maxSamples=Math.max(2, Math.floor(drawW));     // ≈ 1 échantillon / pixel

    // On affiche STRICTEMENT les derniers "maxSamples" points => vraie fenêtre glissante
    const displaySize = Math.min(size, maxSamples);

    // Calcul min/max SUR LA FENÊTRE VISIBILE uniquement
    let idx=head-displaySize; if(idx<0) idx+=N;
    let wMin=Infinity, wMax=-Infinity;
    for(let i=0;i<displaySize;i++){
      const v = data[idx];
      if(v<wMin) wMin=v;
      if(v>wMax) wMax=v;
      if(++idx>=N) idx=0;
    }
    // marge visuelle
    const pad = Math.max(20, 0.08*(wMax - wMin || 1));
    wMin -= pad; wMax += pad;
    if(wMax - wMin < 200){ const mid=(wMin+wMax)/2; wMin=mid-100; wMax=mid+100; }

    // Lissage des bornes pour éviter les “sauts”
    yMin = (1-ALPHA_MM)*yMin + ALPHA_MM*wMin;
    yMax = (1-ALPHA_MM)*yMax + ALPHA_MM*wMax;

    // ligne verticale de “tête”
    if(marginRight>24){
      ctx.strokeStyle='#213252'; ctx.lineWidth=1; ctx.beginPath();
      ctx.moveTo(drawW,0); ctx.lineTo(drawW,H); ctx.stroke();
    }

    // trace
    ctx.strokeStyle='#7fd1ff'; ctx.lineWidth=2; ctx.beginPath();
    const scale=(drawW-1)/(displaySize-1);
    idx=head-displaySize; if(idx<0) idx+=N;
    for(let i=0;i<displaySize;i++){
      const x=i*scale;
      const y=yOf(data[idx]);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
      if(++idx>=N) idx=0;
    }
    ctx.stroke();
  }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

const statusEl=document.getElementById('status');
const bpmEl=document.getElementById('bpm');

function connectWS(){
  const proto=location.protocol==='https:'?'wss':'ws';
  const url=proto+'://'+location.host.replace(/:80$/,'')+':81/';
  const ws=new WebSocket(url);
  ws.onopen = ()=> statusEl.textContent='connected';
  ws.onclose= ()=>{ statusEl.textContent='reconnecting…'; setTimeout(connectWS,1000); };
  ws.onmessage=(ev)=>{
    const t=ev.data.trim();
    if(t.startsWith('FILT=')){
      const raw=parseFloat(t.slice(5));
      if(Number.isFinite(raw)){
        // suppression lente de la composante DC -> recentrage vertical
        dc = (1-ALPHA_DC)*dc + ALPHA_DC*raw;
        push(raw - dc);
      }
    } else if(t.startsWith('BPM=')){
      const b=parseFloat(t.slice(4));
      if(Number.isFinite(b)) bpmEl.textContent=b.toFixed(1);
    }
  };
}
connectWS();
</script></body></html>
)HTML";

// UART: STM32 -> ESP32 on Serial2 RX (GPIO16)
static const int UART_RX_PIN = 16;   // RX2
static const int UART_TX_PIN = 17;   // TX2 (pas utilisé)
static const uint32_t UART_BAUD = 115200;

String lineBuf;

void handleRoot(){ http.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }
void handleIP(){ http.send(200, "text/plain", WiFi.softAPIP().toString()); }

void setup() {
  Serial.begin(115200);
  Serial.println("\n[ECG-ESP32] boot");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  http.on("/", handleRoot);
  http.on("/ip", handleIP);
  http.begin();

  ws.begin();
  ws.onEvent([](uint8_t, WStype_t, uint8_t*, size_t){});

  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  lineBuf.reserve(64);
}

void loop() {
  http.handleClient();
  ws.loop();

  while (Serial2.available()){
    char c=(char)Serial2.read();
    if(c=='\r') continue;
    if(c=='\n'){
      if(lineBuf.length()){
        if (lineBuf.startsWith("FILT=") || lineBuf.startsWith("BPM=")) {
          ws.broadcastTXT(lineBuf);
        }
        lineBuf="";
      }
    }else{
      if(lineBuf.length()<60) lineBuf+=c; else lineBuf="";
    }
  }
}