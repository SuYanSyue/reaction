/*
 * 【光學動力學分析儀 - S3 Zero V30.3 mDNS 滿血回歸完整版】
 * 1. [修復] 強制固定 mDNS 為 reaction.local，方便單機測試與無腦連線。
 * 2. [保留] V30.2 排版修復：檢量線 Data Chips 強制由左至右橫向延伸。
 * 3. [保留] V30.0 / 30.1 功能：下載 CSV、OLED ADC/Abs 標籤正名、三色基準 (I₀)。
 * 4. [保留] V29.1 功能：SPA 架構、橫向排版、自動排序、標準差計算、WebSerial 正常運作。
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <WebSocketsServer.h> 
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Wire.h> 
#include <Adafruit_ADS1X15.h> 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define I2C_SDA 5
#define I2C_SCL 6
#define LED_R 7
#define LED_G 8
#define LED_B 9

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_ADS1115 ads; 

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WiFiManager wm;

String myHostname = "reaction";
volatile int currentADC = 0;
volatile float currentAbs = 0.0;

float vRefR = 20000.0, vRefG = 20000.0, vRefB = 20000.0; 
float activeVRef = 20000.0;
String currentLight = "R"; 

volatile bool isRunning = false;
enum AnalysisState { IDLE, WAITING, ANALYZING, FINISHED };
AnalysisState currentMode = IDLE;
float thresholdStart = 0.20, thresholdEnd = 0.80;

float finalAbsSlope = 0, finalAbsRSq = 0;
float finalAdcSlope = 0, finalAdcRSq = 0;
unsigned long analysisStartTime = 0, runStartTime = 0; 
const int MAX_SAMPLES = 400; 
float timeData[MAX_SAMPLES], absData[MAX_SAMPLES], adcData[MAX_SAMPLES];
int sampleCount = 0;

void setLight(String color) {
  currentLight = color;
  digitalWrite(LED_R, color == "R" ? HIGH : LOW);
  digitalWrite(LED_G, color == "G" ? HIGH : LOW);
  digitalWrite(LED_B, color == "B" ? HIGH : LOW);
  if(color == "R") activeVRef = vRefR;
  else if(color == "G") activeVRef = vRefG;
  else if(color == "B") activeVRef = vRefB;
}

// ==========================================
// 🌟 單頁應用程式 (SPA) HTML
// ==========================================
const char dashboard_html[] PROGMEM = R"=====(
<!DOCTYPE html><html lang="zh-TW"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>光學動力學分析儀 V30.3</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
 :root { --p: #4361ee; --s: #2ecc71; --d: #e63946; --bg: #f4f6f9; }
 html, body { height: 100%; margin: 0; padding: 0; font-family: sans-serif; background: var(--bg); color: #333; overflow-x: hidden; }
 .modal { display: none; position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.6); z-index: 1000; align-items: center; justify-content: center; backdrop-filter: blur(2px); }
 .modal-content { background: #fff; padding: 25px; border-radius: 12px; max-width: 450px; width: 90%; box-shadow: 0 10px 25px rgba(0,0,0,0.2); }
 .code-box { background: #f4f6f9; padding: 12px; border-radius: 6px; font-family: monospace; font-size: 14px; color: var(--d); margin: 6px 0; border: 1px solid #e1e4e8; text-align: center; font-weight: bold; cursor: pointer; user-select: all; word-break: break-all; }
 .btn-home { position: absolute; right: 15px; top: 15px; background: #495057; color: #fff; border: none; padding: 8px 12px; border-radius: 8px; font-weight: bold; cursor: pointer; z-index: 100; box-shadow: 0 2px 5px rgba(0,0,0,0.2); }
 #homePage { display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; padding: 20px; text-align: center; }
 .menu-btn { display: flex; flex-direction: column; align-items: center; justify-content: center; width: 100%; max-width: 380px; padding: 30px; margin: 15px 0; background: #fff; border: 2px solid var(--p); border-radius: 16px; cursor: pointer; transition: 0.3s; box-shadow: 0 4px 6px rgba(0,0,0,0.05); }
 .menu-btn:hover { transform: translateY(-5px); box-shadow: 0 8px 15px rgba(67, 97, 238, 0.2); background: #f4f7ff; }
 .menu-btn h2 { margin: 0 0 10px 0; color: var(--p); font-size: 24px; }
 .app-page { display: none; flex-direction: column; height: 100vh; padding: 15px; box-sizing: border-box; overflow-y: auto; position: relative; }
 .top-bar { display: flex; justify-content: space-between; align-items: center; background: #ffffff; padding: 8px 12px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.05); margin-bottom: 10px; border: 1px solid #e1e4e8; flex-wrap: wrap; gap: 8px; flex: 0 0 auto; padding-right: 100px; }
 .status-badge { font-size: 13px; font-weight: bold; color: #666; flex: 1; }
 .setup-row { display: flex; gap: 8px; align-items: center; justify-content: center; flex: 2; font-size: 13px; font-weight: bold; color: #444; }
 .setup-row input { width: 55px; background: #f8f9fa; border: 1px solid #ccc; padding: 4px; border-radius: 6px; text-align: center; font-weight: bold; font-size: 13px; }
 .rgb-group { display: flex; gap: 10px; width: 100%; margin-bottom: 10px; flex: 0 0 auto; }
 .btn-rgb { flex: 1; padding: 14px 10px; border: none; border-radius: 8px; font-weight: bold; cursor: pointer; color: white; opacity: 0.4; font-size: 15px; }
 .btn-rgb.active { opacity: 1; box-shadow: 0 4px 8px rgba(0,0,0,0.3); transform: scale(1.02); }
 .dashboard { display: grid; grid-template-columns: repeat(2,1fr); gap: 10px; margin-bottom: 10px; flex: 0 0 auto; }
 .card { padding: 10px; border-radius: 8px; text-align: center; display: flex; flex-direction: column; justify-content: center; box-shadow: 0 2px 4px rgba(0,0,0,0.05); }
 .card-adc { border: 3px solid var(--p); background: #f4f7ff; }
 .card-abs { border: 3px solid var(--d); background: #fff4f4; }
 .lbl-row { font-size: 13px; color: #555; display: flex; justify-content: space-around; margin-bottom: 4px; font-weight: bold; }
 .val-row { display: flex; justify-content: space-around; align-items: baseline; margin: 5px 0; }
 .val { font-size: 30px; font-weight: bold; font-family: monospace; width: 33%; text-align: center; }
 .charts { display: flex; flex-direction: row; gap: 10px; flex: 1 1 0%; min-height: 250px; width: 100%; box-sizing: border-box; }
 .chart-box { flex: 1; background: #ffffff; padding: 10px; border-radius: 8px; display: flex; flex-direction: column; border: 1px solid #e1e4e8; min-width: 0; }
 .chart-wrapper { position: relative; flex: 1 1 0%; height: 100%; width: 100%; min-height: 0; }
 .footer { display: flex; flex-direction: row; gap: 8px; padding: 10px 0 5px 0; width: 100%; flex: 0 0 auto; }
 .footer button { flex: 1; padding: 14px 2px; border-radius: 6px; border: none; font-weight: bold; font-size: 15px; cursor: pointer; }
 .sop-card { background: #fff; border-radius: 12px; padding: 20px; margin-bottom: 15px; border: 1px solid #e1e4e8; }
 .step-title { font-size: 18px; font-weight: bold; color: var(--p); margin-bottom: 12px; display: flex; align-items: center; gap: 8px; }
 .step-title span { background: var(--p); color: #fff; width: 24px; height: 24px; border-radius: 50%; display: flex; align-items: center; justify-content: center; font-size: 14px; }
 .btn-sop { padding: 10px 15px; border: none; border-radius: 8px; font-weight: bold; cursor: pointer; font-size: 14px; background: var(--p); color: #fff; }
 .chip-group { display: flex; flex-wrap: wrap; gap: 8px; flex: 1; justify-content: flex-start; }
 .chip { background: #eef2ff; border: 1px solid var(--p); padding: 6px 12px; border-radius: 20px; display: flex; align-items: center; gap: 6px; font-weight: bold; color: var(--p); font-size: 14px;}
 .chip .remove { cursor: pointer; color: var(--d); font-size: 18px; line-height:1;}
 .badge { padding: 4px 10px; border-radius: 12px; font-size: 12px; font-weight: bold; }
 .badge-green { background: #dcfce7; color: #15803d; }
 .badge-red { background: #fee2e2; color: #b91c1c; }
 .cali-table-header { display:flex; font-size:13px; color:#666; font-weight:bold; border-bottom:2px solid #e1e4e8; padding-bottom:8px; margin-bottom:15px; }
 .row-wrap { display:flex; align-items:flex-start; gap:15px; margin-bottom:12px; border-bottom:1px dashed #eee; padding-bottom:12px; }
</style></head><body>

 <div id="serialModal" class="modal"><div class="modal-content"><h3 style="margin-top:0; color:#e63946;">⚠️ 啟用 USB 傳輸權限</h3><p style="font-size:14px; color:#555;">請前往 Chrome 設定頁面並設為 <b>Enabled</b>：</p><div class="code-box">chrome://flags/#unsafely-treat-insecure-origin-as-secure</div><p style="font-size:14px; color:#555;">目前的儀器網址：</p><div class="code-box" id="urlTxt">http://reaction.local</div><div style="text-align:right; margin-top:20px;"><button onclick="document.getElementById('serialModal').style.display='none'" style="padding:10px 20px; background:#6c757d; color:#fff; border:none; border-radius:6px; font-weight:bold; cursor:pointer;">關閉</button></div></div></div>

 <div id="homePage">
   <h1 style="color: #4361ee; margin-bottom: 5px; font-size: 28px;">光學動力學分析儀</h1>
   <p style="color: #666; margin-bottom: 30px; font-weight:bold;" id="homeStatus">偵測連線狀態中...</p>
   <div class="menu-btn" onclick="openApp('K')"><h2>📉 反應速率測定</h2><p>V26.4 旗艦模式：雙圖表即時監測與速率運算</p></div>
   <div class="menu-btn" onclick="openApp('C')"><h2>📏 未知濃度檢量</h2><p>SOP 引導精靈：自動校正、多重取樣、迴歸換算</p></div>
   <div class="footer" id="home-footer" style="max-width: 380px;"></div>
 </div>

 <div id="appK" class="app-page">
   <button class="btn-home" onclick="goHome()">🏠 回主選單</button>
   <div class="top-bar">
     <div id="connModeK" class="status-badge" style="text-align:left;">連線中...</div>
     <div class="setup-row"><span>起始 Abs: <input type="number" id="thS" value="0.20" step="0.05"></span><span>結束: <input type="number" id="thE" value="0.80" step="0.05"></span><button style="background:#495057;color:#fff;border:none;padding:5px 10px;border-radius:4px;cursor:pointer;" onclick="syncParams()">同步</button></div>
     <div id="statusK" class="status-badge" style="text-align:right;">待機 (RAW)</div>
   </div>
   <div class="rgb-group">
     <button id="btnR" class="btn-rgb active" style="background:#e63946;" onclick="switchLight('R')">🔴 紅光 (Red)</button>
     <button id="btnG" class="btn-rgb" style="background:#2ecc71;" onclick="switchLight('G')">🟢 綠光 (Green)</button>
     <button id="btnB" class="btn-rgb" style="background:#4361ee;" onclick="switchLight('B')">🔵 藍光 (Blue)</button>
   </div>
   <div class="dashboard">
    <div class="card card-adc"><div class="lbl-row"><span>即時讀數 (ADC)</span><span>下降速率</span><span>決定係數(R²)</span></div><div class="val-row"><div id="v1" class="val" style="color:var(--p)">00000</div><div id="adcSlope" class="val" style="color:var(--p)">0.0</div><div id="adcRsq" class="val val-rsq" style="color:var(--p)">0.000</div></div></div>
    <div class="card card-abs"><div class="lbl-row"><span>吸光度 (Abs)</span><span>生成速率</span><span>決定係數(R²)</span></div><div class="val-row"><div id="v2" class="val" style="color:var(--d)">0.000</div><div id="absSlope" class="val" style="color:var(--d)">0.000</div><div id="absRsq" class="val val-rsq" style="color:var(--d)">0.000</div></div></div>
   </div>
   <div class="charts"><div class="chart-box"><div class="chart-wrapper"><canvas id="c1"></canvas></div></div><div class="chart-box"><div class="chart-wrapper"><canvas id="c2"></canvas></div></div></div>
   <div class="footer" id="k-footer"></div>
 </div>

 <div id="appC" class="app-page">
   <button class="btn-home" onclick="goHome()">🏠 回主選單</button>
   <h2 style="color:var(--p); margin: 0 0 15px 0;">📏 檢量線製作 SOP</h2>
   <div class="sop-card" style="display: flex; gap: 20px; flex-wrap: wrap; padding: 20px;">
     <div style="flex: 1; min-width: 280px;"><div class="step-title"><span>1</span> 校正空白液 (純水)</div><p style="font-size:13px; color:#666; margin-bottom:15px;">放入純水比色管，校正三種光源基礎亮度。</p><button class="btn-sop" onclick="doBlankStep()">☁️ 開始校準三色基準 (I₀)</button><span id="blankStatus" class="badge" style="margin-left:10px;">尚未校正</span></div>
     <div style="width: 1px; background: #e1e4e8;"></div>
     <div style="flex: 1; min-width: 280px;"><div class="step-title"><span>2</span> 尋找最佳吸收波長</div><p style="font-size:13px; color:#666; margin-bottom:15px;">放入最高濃度標準品，評估感度最高的光源。</p><button class="btn-sop" id="btnScan" disabled onclick="doScanStep()">🌈 自動掃描最佳光源</button><span id="scanStatus" class="badge" style="margin-left:10px;">等待步驟 1</span></div>
   </div>
   <div class="sop-card">
     <div class="step-title"><span>3</span> 建構檢量線 (多重複採樣)</div>
     <div class="cali-table-header"><div style="flex: 0 0 100px;">濃度 (C)</div><div style="flex: 1;">取樣數據 (點擊 ✖ 剔除)</div><div style="flex: 0 0 160px; text-align: right; padding-right:15px;">統計結果</div><div style="width: 30px;"></div></div>
     <div id="caliRows"></div>
     <div style="display:flex; justify-content:space-between; align-items:center; margin-top:15px;"><button class="btn-sop" style="background:var(--warning); color:#333;" onclick="addCaliRow()">➕ 新增濃度列</button><div id="healthScore" style="font-size:14px; font-weight:bold;"></div></div>
   </div>
   <div class="sop-card"><div class="step-title"><span>4</span> 檢量線圖表與迴歸</div>
     <div style="display: flex; gap: 20px; flex-wrap: wrap; align-items: stretch;"><div style="flex: 2; height: 300px; min-width: 300px;"><canvas id="chartC"></canvas></div><div style="flex: 1; min-width: 250px; display: flex; flex-direction: column; justify-content: center;"><div id="regFormula" style="font-family:monospace; background:#f8f9fa; padding:20px; border-radius:12px; border:1px solid #e1e4e8; font-size:16px; line-height:1.8; color:#444;"><span style="color:#999;">尚未生成擬合線...</span></div></div></div>
   </div>
   <div class="sop-card">
     <div class="step-title"><span>5</span> 未知濃度換算</div>
     <div style="display: flex; gap: 15px; align-items: center; flex-wrap: wrap;">
       <button class="btn-sop" style="background:var(--s); padding:12px 20px; font-size:16px;" onclick="captureUnknown()">🧪 讀取未知物 Abs 並換算</button>
       <div id="unkResult" style="font-size:28px; font-weight:bold; color:var(--d); background:#fff4f4; padding:10px 20px; border-radius:8px; border:1px solid #ffccd5; min-width:150px; text-align:center;">C = ???</div>
       <button class="btn-sop" style="background:#6c757d; padding:12px 20px;" onclick="downloadCaliCSV()">💾 下載檢量線 CSV</button>
     </div>
   </div>
 </div>

<script>
 let port, reader, ws, isRunning=false, dataLog=[];
 const isUSB = window.location.pathname.includes('usb');
 let c1K, c2K, chartC;
 let caliData = [{c: 0, samples: []}];
 let regParams = {m:0, b:0};

 const openApp = (app) => {
   document.getElementById('homePage').style.display = 'none';
   document.getElementById('appK').style.display = (app=='K') ? 'flex' : 'none';
   document.getElementById('appC').style.display = (app=='C') ? 'flex' : 'none';
 };
 const goHome = () => { document.getElementById('homePage').style.display = 'flex'; document.getElementById('appK').style.display = 'none'; document.getElementById('appC').style.display = 'none'; };

 const initUI = () => {
   const homeFooter = document.getElementById('home-footer');
   const kFooter = document.getElementById('k-footer');
   if (isUSB) {
     document.getElementById('homeStatus').innerText = "🔌 USB 連線模式";
     homeFooter.innerHTML = `<button style="background:var(--p); color:#fff; flex:1; padding:12px; border-radius:8px; border:none; font-weight:bold; font-size:16px; cursor:pointer;" onclick="initUSB()">🔌 啟動 USB 連線</button><button style="background:#6c757d; color:#fff; flex:1; padding:12px; border-radius:8px; border:none; font-weight:bold; font-size:16px; cursor:pointer; margin-left:10px;" onclick="location.href='/'">📡 切換無線</button>`;
     kFooter.innerHTML = `<button style="background:#f39c12; color:#fff;" onclick="setZero()">☁️ 校準空白</button><button id="btnRun" style="background:var(--s); color:#fff;" onclick="toggleRun()">▶ 開始記錄</button><button style="background:#6c757d; color:#fff;" onclick="downloadCSV()">💾 下載數據</button>`;
   } else {
     document.getElementById('homeStatus').innerText = "📡 無線網路模式";
     homeFooter.innerHTML = `<button style="background:#8b9dc3; color:#fff; flex:1; padding:12px; border-radius:8px; border:none; font-weight:bold; font-size:16px; cursor:pointer;" onclick="location.href='/wifi_config'">⚙️ WiFi 設定</button><button style="background:#6c757d; color:#fff; flex:1; padding:12px; border-radius:8px; border:none; font-weight:bold; font-size:16px; cursor:pointer; margin-left:10px;" onclick="location.href='/usb'">🔌 切換 USB</button>`;
     kFooter.innerHTML = `<button style="background:#f39c12; color:#fff;" onclick="setZero()">☁️ 校準空白</button><button id="btnRun" style="background:var(--s); color:#fff;" onclick="toggleRun()">▶ 開始記錄</button><button style="background:#6c757d; color:#fff;" onclick="downloadCSV()">💾 下載數據</button>`;
   }
   const mkChart = (id, color, max, yTitle) => {
     return new Chart(document.getElementById(id), {
       type:'line', data:{datasets:[{borderColor:color, data:[], borderWidth:2, pointRadius:0, fill:false}]},
       options:{ animation: false, responsive:true, maintainAspectRatio:false, scales:{ x: { type:'linear', title:{display:true, text:'時間 (s)'} }, y: { min:0, max:max, title:{display:true, text:yTitle} } }, plugins:{legend:{display:false}} }
     });
   };
   c1K = mkChart('c1', '#4361ee', 22000, 'ADC');
   c2K = mkChart('c2', '#e63946', 1.0, 'Abs');
   chartC = new Chart(document.getElementById('chartC'), {
     type: 'scatter', data: { datasets: [{ label: '標準點', backgroundColor: '#4361ee', data: [] }, { label: '擬合線', type: 'line', borderColor: '#e63946', data: [], fill: false, pointRadius: 0 }] },
     options: { responsive: true, maintainAspectRatio: false, scales: { x: { title: { display: true, text: '濃度 (C)' } }, y: { title: { display: true, text: '吸光度 (Abs)' } } }, plugins: { legend: { display: false } } }
   });
   renderCaliRows();
 };

 const processData = (line) => {
   const p = line.trim().split(','); if(p.length < 8) return;
   let t = parseFloat(p[0]), adc = parseInt(p[1]), abs = parseFloat(p[2]), absS = parseFloat(p[3]), absR = parseFloat(p[4]), adcS = parseFloat(p[5]), adcR = parseFloat(p[6]), mode = p[7].trim();
   document.getElementById('v1').innerText = adc; document.getElementById('v2').innerText = abs.toFixed(3);
   document.getElementById('absSlope').innerText = absS.toFixed(4); document.getElementById('absRsq').innerText = absR.toFixed(3);
   document.getElementById('adcSlope').innerText = adcS.toFixed(1); document.getElementById('adcRsq').innerText = adcR.toFixed(3);
   let sTxt = (mode=="WAITING") ? "等待 (Abs > "+document.getElementById('thS').value+")..." : (mode=="ANALYZING") ? "紀錄與擬合中..." : (mode=="FINISHED") ? "反應完成" : "待機";
   document.getElementById('statusK').innerText = sTxt;
   document.getElementById('statusK').style.color = (mode=="ANALYZING") ? "#e63946" : "#666";
   if(mode=="ANALYZING" || mode=="WAITING") {
     c1K.data.datasets[0].data.push({x:t, y:adc}); if(c1K.data.datasets[0].data.length>300) c1K.data.datasets[0].data.shift(); c1K.update('none');
     c2K.data.datasets[0].data.push({x:t, y:abs}); if(c2K.data.datasets[0].data.length>300) c2K.data.datasets[0].data.shift(); c2K.update('none');
     dataLog.push({t, adc, abs});
   }
 };

 const doBlankStep = async () => {
   document.getElementById('blankStatus').innerText = "校正中...";
   const res = await fetch('/api/blank_all');
   if(res.ok) { document.getElementById('blankStatus').innerText = "✅ 完成"; document.getElementById('blankStatus').className = "badge badge-green"; document.getElementById('btnScan').disabled = false; }
 };

 const doScanStep = async () => {
   document.getElementById('scanStatus').innerText = "掃描中...";
   const res = await fetch('/api/scan_best'); const data = await res.json();
   document.getElementById('scanStatus').innerText = "✅ 最佳: " + data.best; document.getElementById('scanStatus').className = "badge badge-green";
   ['R','G','B'].forEach(c => document.getElementById('btn'+c).className = (c==data.best)?'btn-rgb active':'btn-rgb');
 };

 const addCaliRow = () => { caliData.push({c: 0, samples: []}); renderCaliRows(); };
 const addSample = (idx) => { caliData[idx].samples.push(parseFloat(document.getElementById('v2').innerText)); renderCaliRows(); };
 const removeSample = (rowIdx, sIdx) => { caliData[rowIdx].samples.splice(sIdx, 1); renderCaliRows(); };
 const removeCaliRow = (idx) => { caliData.splice(idx, 1); renderCaliRows(); };

 const renderCaliRows = () => {
   let html = "";
   caliData.forEach((row, i) => {
     let avg = 0, stdev = 0;
     if(row.samples.length > 0) {
       avg = row.samples.reduce((a,b)=>a+b,0)/row.samples.length;
       if(row.samples.length > 1) {
         let sumSq = row.samples.reduce((a,b) => a + Math.pow(b-avg, 2), 0);
         stdev = Math.sqrt(sumSq / (row.samples.length - 1));
       }
     }
     let chips = row.samples.map((v, si) => `<div class="chip">${v.toFixed(3)}<span class="remove" onclick="removeSample(${i},${si})">×</span></div>`).join('');
     html += `<div class="row-wrap"><div style="flex: 0 0 100px;"><input type="number" value="${row.c}" step="any" onblur="caliData[${i}].c=parseFloat(this.value); caliData.sort((a,b)=>a.c-b.c); renderCaliRows();" style="width:70px; padding:6px; border-radius:6px; border:1px solid #ccc; font-weight:bold;"></div><div style="flex: 1; display: flex; align-items: flex-start; gap: 10px;"><button class="btn-sop" style="padding:6px 12px; background:var(--p); flex-shrink: 0;" onclick="addSample(${i})">＋讀取</button><div class="chip-group" style="margin:0;">${chips}</div></div><div style="flex: 0 0 160px; text-align: right; font-size: 13px; line-height: 1.5; padding-right:15px;">平均: <b style="color:var(--d); font-size:16px;">${row.samples.length>0 ? avg.toFixed(4) : "---"}</b><br>SD: <span style="color:#666;">${stdev.toFixed(4)}</span></div><button style="border:none; background:none; color:#ccc; cursor:pointer; font-size:22px;" onclick="removeCaliRow(${i})">🗑️</button></div>`;
   });
   document.getElementById('caliRows').innerHTML = html; updateReg();
 };

 const updateReg = () => {
   let x = [], y = [], inRangeCount = 0;
   caliData.forEach(d => { if(d.samples.length > 0) { let avg = d.samples.reduce((a,b)=>a+b,0)/d.samples.length; x.push(d.c); y.push(avg); if(avg >= 0.2 && avg <= 1.0) inRangeCount++; } });
   let hs = document.getElementById('healthScore');
   if(x.length < 2) hs.innerHTML = "<span class='badge badge-red'>🔴 至少需要 2 種濃度</span>";
   else if(inRangeCount < 3) hs.innerHTML = "<span class='badge badge-red'>⚠️ 0.2-1.0 內不足 3 個點</span>";
   else hs.innerHTML = "<span class='badge badge-green'>✅ 檢量線品質良好</span>";
   if(x.length >= 2) {
     let n = x.length, sX=0, sY=0, sXY=0, sX2=0, sY2=0;
     for(let i=0; i<n; i++){ sX+=x[i]; sY+=y[i]; sXY+=x[i]*y[i]; sX2+=x[i]*x[i]; sY2+=y[i]*y[i]; }
     let den = (n*sX2 - sX*sX); if(den === 0) return;
     regParams.m = (n*sXY - sX*sY) / den; regParams.b = (sY - regParams.m*sX) / n;
     let r2 = Math.pow((n*sXY - sX*sY), 2) / (den * (n*sY2 - sY*sY));
     document.getElementById('regFormula').innerHTML = `<b>線性迴歸方程式：</b><br><span style="font-size:18px; color:var(--p);">Abs = ${regParams.m.toFixed(4)} * C + (${regParams.b.toFixed(4)})</span><br><b>決定係數：</b><br><span style="font-size:18px; color:var(--d);">R² = ${r2.toFixed(4)}</span>`;
     chartC.data.datasets[0].data = x.map((xv, i) => ({x: xv, y: y[i]}));
     let minX = Math.min(...x), maxX = Math.max(...x);
     chartC.data.datasets[1].data = [{x: minX, y: regParams.m*minX+regParams.b}, {x: maxX, y: regParams.m*maxX+regParams.b}]; chartC.update('none');
   } else { document.getElementById('regFormula').innerHTML = "<span style='color:#999;'>尚未生成擬合線...</span>"; chartC.data.datasets[0].data = []; chartC.data.datasets[1].data = []; chartC.update('none'); }
 };

 const captureUnknown = () => {
   if(regParams.m == 0) { alert("請先完成檢量線！"); return; }
   let abs = parseFloat(document.getElementById('v2').innerText); let conc = (abs - regParams.b) / regParams.m;
   document.getElementById('unkResult').innerText = "C = " + conc.toFixed(4);
 };

 const downloadCaliCSV = () => {
   let csv = "Concentration,Sample1,Sample2,Sample3,Sample4,Sample5,Average,StdDev\n";
   caliData.forEach(row => {
     let avg = row.samples.length > 0 ? (row.samples.reduce((a,b)=>a+b,0)/row.samples.length) : 0;
     let sd = 0; if(row.samples.length>1) { let sq = row.samples.reduce((a,b)=>a+Math.pow(b-avg,2),0); sd = Math.sqrt(sq/(row.samples.length-1)); }
     let s = [null,null,null,null,null]; row.samples.forEach((v,i)=>s[i]=v);
     csv += `${row.c},${s[0]||''},${s[1]||''},${s[2]||''},${s[3]||''},${s[4]||''},${avg.toFixed(4)},${sd.toFixed(4)}\n`;
   });
   let a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([csv], {type: 'text/csv'})); a.download = `cali_data_${new Date().getTime()}.csv`; a.click();
 };

 const switchLight = async (c) => { await fetch('/api/light?c=' + c); ['R','G','B'].forEach(l => document.getElementById('btn'+l).className = (l==c)?'btn-rgb active':'btn-rgb'); };
 const syncParams = () => { fetch(`/sync?s=${document.getElementById('thS').value}&e=${document.getElementById('thE').value}`); alert("已同步"); };
 const setZero = () => { fetch('/blank?val='+document.getElementById('v1').innerText); alert("已校準"); };
 const toggleRun = () => { isRunning = !isRunning; document.getElementById('btnRun').innerText = isRunning ? "⏹ 停止記錄" : "▶ 開始記錄"; document.getElementById('btnRun').style.background = isRunning ? "var(--d)" : "var(--s)"; if(isRunning) { dataLog = []; c1K.data.datasets[0].data = []; c2K.data.datasets[0].data = []; } fetch('/api/toggle?run=' + (isRunning?1:0)); };
 const downloadCSV = () => { let csv = "Time(s),ADC,Absorbance\n"; dataLog.forEach(d => csv += `${d.t.toFixed(1)},${d.adc},${d.abs.toFixed(4)}\n`); let a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([csv], {type: 'text/csv'})); a.download = "kinetic_data.csv"; a.click(); };

 const initUSB = async () => {
   if (!navigator.serial) { document.getElementById('urlTxt').innerText = window.location.origin; document.getElementById('serialModal').style.display='flex'; return; }
   try {
     port = await navigator.serial.requestPort(); await port.open({baudRate:115200});
     document.getElementById('connModeK').innerText = "🔌 USB 已連線";
     const decoder = new TextDecoderStream(); port.readable.pipeTo(decoder.writable);
     reader = decoder.readable.getReader(); let buf=""; 
     while(true) { const {value, done} = await reader.read(); if(done) break; buf += value; let lines = buf.split('\n'); buf = lines.pop(); for(let line of lines) processData(line); }
   } catch(e) { if(e.name === "SecurityError") { document.getElementById('urlTxt').innerText = window.location.origin; document.getElementById('serialModal').style.display='flex'; } }
 };

 window.onload = () => { initUI(); if(!isUSB) { ws = new WebSocket('ws://'+window.location.hostname+':81/'); ws.onopen = () => { document.getElementById('connModeK').innerText = "📡 WiFi 已連線"; }; ws.onmessage = (e) => { e.data.split('|').forEach(row => processData(row)); }; } };
</script></body></html>
)=====";

const char config_html[] PROGMEM = R"=====(
<!DOCTYPE HTML><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<style> body{font-family:sans-serif;background:#f4f6f9;padding:20px;text-align:center;} .card{background:#fff;padding:30px;border-radius:12px;display:inline-block;width:100%;max-width:400px;text-align:left;} input{width:calc(100% - 20px); padding:10px; margin-top:8px; margin-bottom:15px; border:1px solid #ccc; border-radius:6px;} </style>
<body><div class="card"><h2>📡 WiFi 設定</h2><div id="list" style="margin-bottom:20px;">掃描中...</div><form action="/api/save_wifi" method="POST">SSID:<br><input type="text" name="ssid" id="s" required><br>密碼:<br><input type="password" name="password" id="p"><br><input type="submit" value="儲存並重啟" style="width:100%;padding:15px;background:#2ecc71;color:#fff;border:none;border-radius:8px;font-weight:bold;"></form></div>
<script> const fetchAp = () => { fetch('/api/scan_list').then(r=>r.json()).then(d=>{ let h=""; d.forEach(ap=>{h+=`<div style="cursor:pointer;padding:10px;border-bottom:1px solid #eee;color:#4361ee;" onclick="document.getElementById('s').value='${ap.ssid}'">📶 ${ap.ssid} (${ap.rssi}dBm)</div>`;}); document.getElementById('list').innerHTML=h; }); }; window.onload = fetchAp; </script></body></html>
)=====";

void setup() {
  Serial.begin(115200); 
  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  Wire.begin(I2C_SDA, I2C_SCL); 
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println("OLED Fail");
  display.clearDisplay(); display.display();

  // 🌟 V30.3 修復：強制固定 mDNS 網址為 reaction.local
  myHostname = "reaction";
  WiFi.setHostname(myHostname.c_str());

  ads.setGain(GAIN_ONE); ads.begin();
  WiFi.mode(WIFI_AP_STA); 
  wm.autoConnect("REACTION_MONITOR");
  WiFi.setSleep(false);
  
  if (MDNS.begin(myHostname.c_str())) MDNS.addService("http", "tcp", 80);

  server.on("/", [](){ server.send_P(200, "text/html", dashboard_html); });
  server.on("/usb", [](){ server.send_P(200, "text/html", dashboard_html); });
  server.on("/wifi_config", [](){ server.send_P(200, "text/html", config_html); });
  
  server.on("/api/blank_all", [](){
    setLight("R"); delay(300); vRefR = ads.readADC_SingleEnded(0);
    setLight("G"); delay(300); vRefG = ads.readADC_SingleEnded(0);
    setLight("B"); delay(300); vRefB = ads.readADC_SingleEnded(0);
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  });

  server.on("/api/scan_best", [](){
    float maxAbs = -1.0; String best = "R";
    String colors[] = {"R", "G", "B"};
    for(int i=0; i<3; i++) {
      setLight(colors[i]); delay(300);
      int adc = ads.readADC_SingleEnded(0);
      float abs = (adc > 0) ? log10(activeVRef / (float)adc) : 0;
      if(abs > maxAbs && abs < 2.0) { maxAbs = abs; best = colors[i]; }
    }
    setLight(best);
    server.send(200, "application/json", "{\"best\":\""+best+"\"}");
  });

  server.on("/api/light", [](){ if(server.hasArg("c")) setLight(server.arg("c")); server.send(200); });
  server.on("/blank", [](){ if(server.hasArg("val")) activeVRef = server.arg("val").toFloat(); server.send(200); });
  server.on("/sync", [](){ if(server.hasArg("s")) thresholdStart = server.arg("s").toFloat(); if(server.hasArg("e")) thresholdEnd = server.arg("e").toFloat(); server.send(200); });
  
  server.on("/api/toggle", [](){ 
    isRunning = (server.arg("run")=="1"); 
    currentMode = isRunning ? WAITING : IDLE;
    if(isRunning) { sampleCount = 0; runStartTime = millis(); finalAbsSlope=0; finalAbsRSq=0; finalAdcSlope=0; finalAdcRSq=0; }
    server.send(200); 
  });
  
  server.on("/api/scan_list", [](){ 
    int n = WiFi.scanNetworks(); JsonDocument d; JsonArray arr = d.to<JsonArray>();
    for(int i=0; i<n; ++i){ JsonObject o=arr.add<JsonObject>(); o["ssid"]=WiFi.SSID(i); o["rssi"]=WiFi.RSSI(i); }
    String j; serializeJson(d, j); server.send(200, "application/json", j); 
  });
  server.on("/api/save_wifi", HTTP_POST, [](){ 
    WiFi.begin(server.arg("ssid").c_str(), server.arg("password").c_str()); 
    server.send(200, "text/html", "Saved! Restarting..."); delay(2000); ESP.restart(); 
  });

  server.begin(); webSocket.begin(); 
}

void loop() {
  server.handleClient(); webSocket.loop();
  static unsigned long lastData = 0, lastOLED = 0, lastWiFi = 0;
  static String wifiBatch = "";

  if (millis() - lastData >= 100) { 
    lastData = millis();
    currentADC = ads.readADC_SingleEnded(0);
    currentAbs = (activeVRef > 0 && currentADC > 0) ? log10(activeVRef / (float)currentADC) : 0;
    if(currentAbs < 0) currentAbs = 0;

    float t = 0.0;
    if (isRunning) {
      t = (millis() - runStartTime) / 1000.0;
      if (currentMode == WAITING) {
        bool trigger = (thresholdStart < thresholdEnd) ? (currentAbs > thresholdStart) : (currentAbs < thresholdStart);
        if (trigger) { analysisStartTime = millis(); currentMode = ANALYZING; }
      }
      else if (currentMode == ANALYZING && sampleCount < MAX_SAMPLES) {
         timeData[sampleCount] = (millis() - analysisStartTime)/1000.0;
         absData[sampleCount] = currentAbs; adcData[sampleCount] = (float)currentADC;
         sampleCount++;
         if (sampleCount > 5) {
            float sX=0, sY_abs=0, sY_adc=0, sXY_abs=0, sXY_adc=0, sX2=0, sY2_abs=0, sY2_adc=0;
            for(int i=0; i<sampleCount; i++){ sX+=timeData[i]; sX2+=timeData[i]*timeData[i]; sY_abs+=absData[i]; sXY_abs+=timeData[i]*absData[i]; sY2_abs+=absData[i]*absData[i]; sY_adc+=adcData[i]; sXY_adc+=timeData[i]*adcData[i]; sY2_adc+=adcData[i]*adcData[i]; }
            float denX = (sampleCount*sX2 - sX*sX); 
            if(denX != 0) {
              float num_abs = (sampleCount*sXY_abs - sX*sY_abs); finalAbsSlope = abs(num_abs / denX);
              float denY_abs = (sampleCount*sY2_abs - sY_abs*sY_abs); if(denY_abs != 0) finalAbsRSq = (num_abs * num_abs) / (denX * denY_abs);
              float num_adc = (sampleCount*sXY_adc - sX*sY_adc); finalAdcSlope = abs(num_adc / denX);
              float denY_adc = (sampleCount*sY2_adc - sY_adc*sY_adc); if(denY_adc != 0) finalAdcRSq = (num_adc * num_adc) / (denX * denY_adc);
            }
         }
         bool finish = (thresholdStart < thresholdEnd) ? (currentAbs > thresholdEnd) : (currentAbs < thresholdEnd);
         if (finish) currentMode = FINISHED;
      }
    }
    String stStr = (currentMode==WAITING)?"WAITING":(currentMode==ANALYZING)?"ANALYZING":(currentMode==FINISHED)?"FINISHED":"IDLE";
    String out = String(t,2)+","+String(currentADC)+","+String(currentAbs,3)+","+String(finalAbsSlope,4)+","+String(finalAbsRSq,4)+","+String(finalAdcSlope,2)+","+String(finalAdcRSq,4)+","+stStr;
    Serial.println(out); 
    if (wifiBatch != "") wifiBatch += "|"; wifiBatch += out;
  }
  if (millis() - lastWiFi >= 300) { lastWiFi = millis(); if(wifiBatch!="") { webSocket.broadcastTXT(wifiBatch); wifiBatch=""; } }
  if (millis() - lastOLED >= 500) { 
    lastOLED = millis(); 
    display.clearDisplay(); display.setTextColor(SSD1306_WHITE); display.setTextSize(1); display.setCursor(0, 0);
    if(WiFi.status() == WL_CONNECTED) { display.print("IP:"); display.print(WiFi.localIP().toString()); }
    else display.print("USB Mode");
    display.print(" ["); display.print(currentLight); display.print("]");
    display.setTextSize(2);
    display.setCursor(0, 16); display.print("ADC:"); display.print(currentADC);
    display.setCursor(0, 38); display.print("Abs:"); display.print(currentAbs, 3);
    display.setTextSize(1); display.setCursor(0, 56);
    display.print(currentMode == ANALYZING ? ">> ANALYZING <<" : (currentMode == WAITING ? "WAITING..." : "READY"));
    display.display();
  }
}

