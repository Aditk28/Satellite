#!/usr/bin/env python3
"""
live_monitor.py -- self-contained live monitor for the reaction wheel.

The web page is embedded in this file, so this is the only file you need.

Owns the serial port (USB or HC-05), parses the 10 Hz HOLD telemetry stream,
and serves live_monitor.html plus a small JSON API to the browser.

This is capture_calibration.py stripped to what a live view needs: port
handling and line parsing. No CSV state machine, no capture blocks, no
metadata parsing -- those stay in capture_calibration.py for logged runs.

    py -m pip install pyserial
    py live_monitor.py                 # lists ports, picks the only one
    py live_monitor.py --port COM7
    py live_monitor.py --port COM7 --baud 115200 --http 8000

Then open http://localhost:8000

TELEMETRY NOTE: the firmware only streams while in HOLD mode (CTRL_HOLD, not
capturing), at TELEM_PERIOD_MS = 100 -> 10 Hz. That is enough for the numbers
but choppy for video. For recording, set TELEM_PERIOD_MS to 50 (20 Hz) or 33
(30 Hz) in heading_control.cpp. The browser animates between samples either
way, so the instrument stays smooth; the strip charts get denser.

A T<deg> step capture SUSPENDS the stream and dumps a 200 Hz CSV block at the
end. Those rows have 7 fields and are ignored here by field count. Use H<deg>
for anything you want to watch live.
"""

import argparse
import json
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not found.  py -m pip install pyserial")


HERE = Path(__file__).resolve().parent
MAX_SAMPLES = 4000          # ~6.7 min at 10 Hz
MAX_LOGS = 400


PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Reaction wheel — attitude monitor</title>
<style>
  :root{
    --ink:#0B1013;
    --panel:#131C21;
    --panel-2:#0F171B;
    --line:#24343B;
    --signal:#5FD3E0;      /* platform / heading */
    --ember:#E8964A;       /* wheel / momentum */
    --alert:#E2574C;       /* saturation / stop */
    --text:#DCE8ED;
    --muted:#7E939C;
    --mono: ui-monospace, "SF Mono", "Cascadia Mono", Menlo, Consolas, monospace;
    --sans: "Inter", "Segoe UI", system-ui, -apple-system, sans-serif;
  }
  *{box-sizing:border-box}
  html,body{margin:0;height:100%}
  body{
    background:var(--ink); color:var(--text); font-family:var(--sans);
    font-size:14px; line-height:1.5; -webkit-font-smoothing:antialiased;
  }
  .eyebrow{
    font-size:10px; letter-spacing:.16em; text-transform:uppercase;
    color:var(--muted); font-weight:500;
  }
  .num{font-family:var(--mono); font-variant-numeric:tabular-nums}

  header{
    display:flex; align-items:baseline; gap:18px; flex-wrap:wrap;
    padding:16px 22px; border-bottom:1px solid var(--line);
  }
  header h1{font-size:15px; font-weight:500; margin:0; letter-spacing:.01em}
  .link-state{margin-left:auto; display:flex; align-items:center; gap:8px}
  .dot{width:7px;height:7px;border-radius:50%;background:var(--alert)}
  .dot.on{background:var(--signal)}

  main{
    display:grid; grid-template-columns: minmax(340px,1fr) 300px;
    gap:1px; background:var(--line); min-height:calc(100vh - 53px);
  }
  section{background:var(--ink); padding:20px 22px}
  @media (max-width:860px){ main{grid-template-columns:1fr} }

  /* ---- instrument ---- */
  #adi{width:100%; max-width:420px; display:block; margin:0 auto}

  .readout{
    display:grid; grid-template-columns:repeat(3,1fr); gap:1px;
    background:var(--line); border:1px solid var(--line); margin-top:18px;
  }
  .cell{background:var(--panel); padding:9px 11px}
  .cell .v{font-family:var(--mono); font-size:19px; font-variant-numeric:tabular-nums}
  .cell .u{color:var(--muted); font-size:11px; margin-left:2px}
  .cell.sig .v{color:var(--signal)}
  .cell.emb .v{color:var(--ember)}
  .cell.warn .v{color:var(--alert)}

  /* ---- charts ---- */
  .chart-wrap{margin-top:20px}
  canvas.strip{width:100%; height:96px; display:block;
    background:var(--panel-2); border:1px solid var(--line)}
  .chart-head{display:flex; justify-content:space-between; align-items:baseline; margin-bottom:5px}

  /* ---- controls ---- */
  .group{margin-bottom:22px}
  .group > .eyebrow{display:block; margin-bottom:8px}
  .grid{display:grid; grid-template-columns:repeat(4,1fr); gap:5px}
  button{
    font-family:var(--sans); font-size:12px; color:var(--text);
    background:var(--panel); border:1px solid var(--line);
    padding:8px 4px; cursor:pointer; transition:background .12s,border-color .12s;
  }
  button:hover{background:#1B262C; border-color:#33474F}
  button:active{background:#22313A}
  button:focus-visible{outline:2px solid var(--signal); outline-offset:1px}
  button.wide{grid-column:span 4}
  button.half{grid-column:span 2}
  button.go{border-color:#2F5A62; color:var(--signal)}
  button.stop{border-color:#5A2E2B; color:var(--alert)}
  .row{display:flex; gap:5px}
  input[type=text],input[type=number]{
    flex:1; min-width:0; font-family:var(--mono); font-size:13px;
    color:var(--text); background:var(--panel-2);
    border:1px solid var(--line); padding:8px 9px;
  }
  input:focus{outline:none; border-color:var(--signal)}
  .hint{color:var(--muted); font-size:11px; margin-top:7px}

  #console{
    font-family:var(--mono); font-size:11px; line-height:1.65;
    background:var(--panel-2); border:1px solid var(--line);
    height:150px; overflow-y:auto; padding:8px 10px; white-space:pre-wrap;
  }
  #console .out{color:var(--signal)}
  #console .warn{color:var(--ember)}
  #console .err{color:var(--alert)}
  #console .t{color:var(--muted)}
</style>
</head>
<body>

<header>
  <h1>Reaction wheel attitude monitor</h1>
  <span class="eyebrow">Nucleo-F446RE &middot; 200 Hz loop</span>
  <div class="link-state">
    <span class="dot" id="dot"></span>
    <span class="eyebrow" id="linkText">connecting</span>
  </div>
</header>

<main>
  <section>
    <canvas id="adi" width="840" height="840"></canvas>

    <div class="readout">
      <div class="cell sig"><span class="eyebrow">Heading</span>
        <div><span class="v" id="rHeading">—</span><span class="u">deg</span></div></div>
      <div class="cell"><span class="eyebrow">Target</span>
        <div><span class="v" id="rTarget">—</span><span class="u">deg</span></div></div>
      <div class="cell" id="cErr"><span class="eyebrow">Error</span>
        <div><span class="v" id="rErr">—</span><span class="u">deg</span></div></div>

      <div class="cell emb"><span class="eyebrow">Wheel</span>
        <div><span class="v" id="rWw">—</span><span class="u">rad/s</span></div></div>
      <div class="cell"><span class="eyebrow">Platform rate</span>
        <div><span class="v" id="rWp">—</span><span class="u">rad/s</span></div></div>
      <div class="cell"><span class="eyebrow">Command</span>
        <div><span class="v" id="rU">—</span><span class="u">V</span></div></div>
    </div>

    <div class="chart-wrap">
      <div class="chart-head">
        <span class="eyebrow">Heading vs target</span>
        <span class="eyebrow num" id="axHead">±180°</span>
      </div>
      <canvas class="strip" id="chHeading" width="1200" height="192"></canvas>
    </div>

    <div class="chart-wrap">
      <div class="chart-head">
        <span class="eyebrow">Wheel speed &middot; saturation at 45</span>
        <span class="eyebrow num" id="axWheel">±50 rad/s</span>
      </div>
      <canvas class="strip" id="chWheel" width="1200" height="192"></canvas>
    </div>
  </section>

  <section>
    <div class="group">
      <span class="eyebrow">Hold</span>
      <div class="row">
        <input type="number" id="holdDeg" placeholder="degrees" step="1" value="0">
        <button class="go" id="btnHold" style="flex:0 0 68px">Hold</button>
      </div>
      <div class="grid" style="margin-top:5px">
        <button class="half" id="btnHoldZero">Hold at 0</button>
        <button class="half" id="btnHoldHere">Hold here</button>
      </div>
      <p class="hint"><span class="num">H</span> holds a heading and keeps
        telemetry streaming. Everything above sends it too — the board only
        streams while holding, so send one to start the live view.</p>
    </div>

    <div class="group">
      <span class="eyebrow">Slew to heading</span>
      <div class="grid">
        <button data-h="-180">−180</button>
        <button data-h="-90">−90</button>
        <button data-h="-45">−45</button>
        <button data-h="0">0</button>
        <button data-h="45">+45</button>
        <button data-h="90">+90</button>
        <button data-h="135">+135</button>
        <button data-h="180">+180</button>
      </div>
      <div class="row" style="margin-top:5px">
        <input type="number" id="customDeg" placeholder="degrees" step="1">
        <button class="go" id="btnGo" style="flex:0 0 68px">Go</button>
      </div>
    </div>

    <div class="group">
      <span class="eyebrow">Reference</span>
      <div class="grid">
        <button class="half" id="btnZero">Zero here</button>
        <button class="half" id="btnBias">Rebias gyro</button>
      </div>
      <p class="hint">Zero before a run. Rebias needs the platform completely
        still for about a second.</p>
    </div>

    <div class="group">
      <span class="eyebrow">Logged step capture</span>
      <div class="row">
        <input type="number" id="capDeg" placeholder="degrees" step="1">
        <button id="btnCap" style="flex:0 0 68px">Run</button>
      </div>
      <p class="hint">Sends <span class="num">T</span> and records at 200 Hz.
        The live stream pauses until the capture dumps, then resumes.</p>
    </div>

    <div class="group">
      <span class="eyebrow">Controller</span>
      <div class="grid">
        <button class="half stop" id="btnStop">Stop</button>
        <button class="half" id="btnResume">Resume</button>
        <button class="wide" id="btnStatus">Print status</button>
      </div>
    </div>

    <div class="group">
      <span class="eyebrow">Raw command</span>
      <div class="row">
        <input type="text" id="rawCmd" placeholder="e.g. F0.95" autocomplete="off">
        <button id="btnRaw" style="flex:0 0 68px">Send</button>
      </div>
    </div>

    <div class="group" style="margin-bottom:0">
      <span class="eyebrow">Board output</span>
      <div id="console"></div>
    </div>
  </section>
</main>

<script>
// ---------------------------------------------------------------- state
const WHEEL_SAT = 45;          // rad/s, matches WHEEL_SAT_LIMIT in firmware
const HIST_SEC  = 30;          // strip chart window

let since = 0, logSince = 0;
let hist = [];                 // {t, theta, target, ww, u}
let live = {theta:0, target:0, wp:0, ww:0, alpha:0, u:0, stamp:0};
let shown = {theta:0, target:0};   // interpolated for display
let wheelPhase = 0;                // integrated client-side for smooth spin
let lastFrame = performance.now();
let lastSampleAt = 0;
let everSeen = false;
let stale = false;             // firmware stopped streaming: X, or a T capture

// ---------------------------------------------------------------- comms
async function send(cmd){
  try{
    await fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
                         body:JSON.stringify({cmd})});
  }catch(e){ pushLog('link error: ' + e.message, 'err'); }
}

let polling = false;
async function poll(){
  if(polling) return;          // never let two requests race: overlapping polls
  polling = true;              // read the same cursor and duplicate every line
  try{
    const r = await fetch(`/stream?since=${since}&logs=${logSince}`);
    const d = await r.json();

    document.getElementById('dot').classList.toggle('on', d.connected);
    document.getElementById('linkText').textContent =
      !d.connected ? 'no link' : (stale ? d.port + ' · idle' : d.port);

    for(const s of d.samples){
      since = s.n; everSeen = true;
      live = {theta:s.theta, target:s.target, wp:s.wp, ww:s.ww,
              alpha:s.alpha, u:s.u, stamp:performance.now()};
      lastSampleAt = performance.now();
      hist.push({t:s.t, theta:s.theta, target:s.target, ww:s.ww, u:s.u});
    }
    if(hist.length){
      const cutoff = hist[hist.length-1].t - HIST_SEC;
      while(hist.length && hist[0].t < cutoff) hist.shift();
    }
    for(const g of d.logs){ logSince = g.n; pushLog(g.text); }
  }catch(e){
    document.getElementById('dot').classList.remove('on');
    document.getElementById('linkText').textContent = 'server down';
  }finally{ polling = false; }
}
setInterval(poll, 80);

// ---------------------------------------------------------------- console
const box = document.getElementById('console');
function pushLog(text, forced){
  let cls = forced || 'out';
  if(!forced){
    if(/STALL|saturation|abort|error|not connected/i.test(text)) cls = 'err';
    else if(/backing off|released|hold|stop/i.test(text))        cls = 'warn';
    else if(text.startsWith('>>'))                               cls = 'out';
    else                                                         cls = 't';
  }
  const line = document.createElement('div');
  line.className = cls;
  line.textContent = text;
  box.appendChild(line);
  while(box.childElementCount > 300) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

// ---------------------------------------------------------------- helpers
const wrap = d => ((d + 180) % 360 + 360) % 360 - 180;
const rad  = d => d * Math.PI / 180;
function lerpAngle(a, b, k){ return a + wrap(b - a) * k; }

// ---------------------------------------------------------------- instrument
const adi = document.getElementById('adi');
const ax  = adi.getContext('2d');

function drawADI(){
  const S = adi.width, C = S/2;
  ax.clearRect(0,0,S,S);
  ax.save(); ax.translate(C,C);

  const rOuter = C*0.90, rTick = C*0.80, rWheel = C*0.50;

  // --- momentum arc: how much of the saturation budget is spent -----------
  const frac = Math.min(Math.abs(live.ww)/WHEEL_SAT, 1);
  ax.lineWidth = 14; ax.lineCap = 'butt';
  ax.strokeStyle = '#1A262C';
  ax.beginPath(); ax.arc(0,0,rOuter+18,0,Math.PI*2); ax.stroke();
  if(frac > 0.002){
    const dir = Math.sign(live.ww) || 1;
    ax.strokeStyle = frac > 0.8 ? '#E2574C' : '#E8964A';
    ax.beginPath();
    ax.arc(0,0,rOuter+18, -Math.PI/2, -Math.PI/2 + dir*frac*Math.PI*2, dir<0);
    ax.stroke();
  }

  // --- outer bezel and degree ticks ---------------------------------------
  ax.strokeStyle = '#24343B'; ax.lineWidth = 1.5;
  ax.beginPath(); ax.arc(0,0,rOuter,0,Math.PI*2); ax.stroke();

  ax.save(); ax.rotate(rad(-shown.theta));      // ring turns with the platform
  for(let d = 0; d < 360; d += 10){
    const major = d % 30 === 0;
    const a = rad(d - 90);
    const r1 = major ? rTick - 26 : rTick - 13;
    ax.strokeStyle = major ? '#41585F' : '#2A3B42';
    ax.lineWidth = major ? 2 : 1;
    ax.beginPath();
    ax.moveTo(Math.cos(a)*r1, Math.sin(a)*r1);
    ax.lineTo(Math.cos(a)*rTick, Math.sin(a)*rTick);
    ax.stroke();
    if(major){
      ax.save();
      ax.translate(Math.cos(a)*(rTick-46), Math.sin(a)*(rTick-46));
      ax.rotate(a + Math.PI/2);
      ax.fillStyle = '#7E939C';
      ax.font = '500 22px ui-monospace, Menlo, Consolas, monospace';
      ax.textAlign = 'center'; ax.textBaseline = 'middle';
      ax.fillText(((d>180)?d-360:d).toString(), 0, 0);
      ax.restore();
    }
  }
  ax.restore();

  // --- target bug ---------------------------------------------------------
  const ta = rad(wrap(shown.target - shown.theta) - 90);
  ax.fillStyle = '#5FD3E0';
  ax.beginPath();
  ax.moveTo(Math.cos(ta)*(rOuter-4),  Math.sin(ta)*(rOuter-4));
  ax.lineTo(Math.cos(ta-0.030)*(rOuter-30), Math.sin(ta-0.030)*(rOuter-30));
  ax.lineTo(Math.cos(ta+0.030)*(rOuter-30), Math.sin(ta+0.030)*(rOuter-30));
  ax.closePath(); ax.fill();

  // --- flywheel: spokes make the spin legible -----------------------------
  ax.save(); ax.rotate(wheelPhase);
  const g = ax.createRadialGradient(0,0,rWheel*0.15, 0,0,rWheel);
  g.addColorStop(0, '#182228'); g.addColorStop(1, '#101A1F');
  ax.fillStyle = g;
  ax.beginPath(); ax.arc(0,0,rWheel,0,Math.PI*2); ax.fill();
  ax.strokeStyle = '#2E4149'; ax.lineWidth = 2;
  ax.beginPath(); ax.arc(0,0,rWheel,0,Math.PI*2); ax.stroke();

  const spokeAlpha = stale ? 0.12
                   : Math.min(0.25 + Math.abs(live.ww)/WHEEL_SAT*0.75, 1);
  ax.strokeStyle = `rgba(232,150,74,${spokeAlpha})`;
  ax.lineWidth = 3;
  for(let i = 0; i < 6; i++){
    const a = i * Math.PI/3;
    ax.beginPath();
    ax.moveTo(Math.cos(a)*rWheel*0.22, Math.sin(a)*rWheel*0.22);
    ax.lineTo(Math.cos(a)*rWheel*0.93, Math.sin(a)*rWheel*0.93);
    ax.stroke();
  }
  ax.fillStyle = '#0B1013';
  ax.beginPath(); ax.arc(0,0,rWheel*0.19,0,Math.PI*2); ax.fill();
  ax.strokeStyle = '#3A5058'; ax.lineWidth = 2; ax.stroke();
  ax.restore();

  // --- fixed nose marker: the platform's own forward direction -------------
  ax.strokeStyle = stale ? '#2F5A62' : '#5FD3E0';
  ax.lineWidth = 3; ax.lineCap = 'round';
  ax.beginPath();
  ax.moveTo(0, -rWheel - 10); ax.lineTo(0, -rOuter + 30);
  ax.stroke();

  // --- centre numerals -----------------------------------------------------
  ax.fillStyle = !everSeen ? '#41585F' : (stale ? '#5E7078' : '#DCE8ED');
  ax.font = '500 62px ui-monospace, Menlo, Consolas, monospace';
  ax.textAlign = 'center'; ax.textBaseline = 'alphabetic';
  ax.fillText(everSeen ? wrap(live.theta).toFixed(1)+'\u00B0' : '——', 0, 14);
  ax.fillStyle = stale ? '#E8964A' : '#7E939C';
  ax.font = '500 17px ui-monospace, Menlo, Consolas, monospace';
  ax.fillText(!everSeen ? 'awaiting telemetry'
              : stale   ? 'stream idle — last known'
              :           `${live.ww.toFixed(1)} rad/s`, 0, 44);

  // dim the whole face while nothing is arriving
  if(stale && everSeen){
    ax.globalCompositeOperation = 'source-over';
    ax.fillStyle = 'rgba(11,16,19,0.45)';
    ax.beginPath(); ax.arc(0,0,C,0,Math.PI*2); ax.fill();
    ax.fillStyle = '#E8964A';
    ax.font = '500 15px ui-monospace, Menlo, Consolas, monospace';
    ax.fillText('STOPPED  ·  send Hold to resume', 0, C*0.72);
  }

  ax.restore();
}

// ---------------------------------------------------------------- strips
function drawStrip(canvas, series, opts){
  const c = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height, pad = 8;
  c.clearRect(0,0,W,H);
  if(hist.length < 2) return;

  const t1 = hist[hist.length-1].t, t0 = t1 - HIST_SEC;
  const span = opts.span;
  const X = t => pad + (t - t0)/HIST_SEC * (W - pad*2);
  const Y = v => H/2 - (v/span) * (H/2 - pad);

  // zero rule and limit rules
  c.strokeStyle = '#24343B'; c.lineWidth = 1;
  c.beginPath(); c.moveTo(pad, Y(0)); c.lineTo(W-pad, Y(0)); c.stroke();
  for(const lim of (opts.limits || [])){
    c.strokeStyle = '#3A2523'; c.setLineDash([5,5]);
    c.beginPath(); c.moveTo(pad, Y(lim)); c.lineTo(W-pad, Y(lim)); c.stroke();
    c.beginPath(); c.moveTo(pad, Y(-lim)); c.lineTo(W-pad, Y(-lim)); c.stroke();
    c.setLineDash([]);
  }

  for(const s of series){
    c.strokeStyle = s.color; c.lineWidth = s.width || 2;
    if(s.dash) c.setLineDash(s.dash);
    c.beginPath();
    let started = false;
    for(const p of hist){
      const v = s.pick(p);
      if(v === null || !isFinite(v)) continue;
      const x = X(p.t), y = Y(Math.max(-span, Math.min(span, v)));
      started ? c.lineTo(x,y) : (c.moveTo(x,y), started = true);
    }
    c.stroke(); c.setLineDash([]);
  }
}

// ---------------------------------------------------------------- loop
function frame(now){
  const dt = Math.min((now - lastFrame)/1000, 0.1);
  lastFrame = now;

  // ease displayed heading toward the last sample so 10 Hz reads as smooth
  const k = 1 - Math.exp(-dt * 12);
  shown.theta  = lerpAngle(shown.theta,  live.theta,  k);
  shown.target = lerpAngle(shown.target, live.target, k);

  // Stream only runs in HOLD mode, so X or a T capture stops it. Freeze the
  // spin rather than animating stale data as though it were live.
  stale = everSeen && (now - lastSampleAt > 1200);
  if(!stale) wheelPhase = (wheelPhase + live.ww * dt) % (Math.PI*2);

  drawADI();

  const err = wrap(live.target - live.theta);
  if(everSeen){
    document.getElementById('rHeading').textContent = wrap(live.theta).toFixed(2);
    document.getElementById('rTarget').textContent  = wrap(live.target).toFixed(1);
    document.getElementById('rErr').textContent     = err.toFixed(2);
    document.getElementById('rWw').textContent      = live.ww.toFixed(2);
    document.getElementById('rWp').textContent      = live.wp.toFixed(3);
    document.getElementById('rU').textContent       = live.u.toFixed(2);
    document.getElementById('cErr').classList.toggle('warn', Math.abs(err) > 2);
    document.getElementById('cErr').classList.toggle('sig',  Math.abs(err) <= 2);
    document.querySelector('.cell.emb').classList.toggle('warn',
      Math.abs(live.ww) > WHEEL_SAT*0.8);
  }

  const headSpan = Math.max(30, Math.min(180,
    hist.reduce((m,p)=>Math.max(m, Math.abs(wrap(p.theta)), Math.abs(wrap(p.target))), 0) * 1.25));
  document.getElementById('axHead').textContent = `±${headSpan.toFixed(0)}°`;
  drawStrip(document.getElementById('chHeading'), [
    {pick:p=>wrap(p.target), color:'#3A5A60', width:2, dash:[6,4]},
    {pick:p=>wrap(p.theta),  color:'#5FD3E0', width:2.5},
  ], {span: headSpan});

  drawStrip(document.getElementById('chWheel'), [
    {pick:p=>p.ww, color:'#E8964A', width:2.5},
  ], {span: 50, limits:[WHEEL_SAT]});

  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

// ---------------------------------------------------------------- controls
document.querySelectorAll('[data-h]').forEach(b =>
  b.addEventListener('click', () => send('H' + b.dataset.h)));

const go = () => {
  const v = document.getElementById('customDeg').value;
  if(v !== '') send('H' + v);
};
document.getElementById('btnGo').addEventListener('click', go);
document.getElementById('customDeg').addEventListener('keydown',
  e => { if(e.key === 'Enter') go(); });

document.getElementById('btnZero')  .addEventListener('click', () => send('Z'));
document.getElementById('btnBias')  .addEventListener('click', () => send('B'));
document.getElementById('btnStop')  .addEventListener('click', () => send('X'));
document.getElementById('btnResume').addEventListener('click', () => send('R'));
document.getElementById('btnStatus').addEventListener('click', () => send('G'));

const hold = () => {
  const v = document.getElementById('holdDeg').value;
  if(v !== '') send('H' + v);
};
document.getElementById('btnHold').addEventListener('click', hold);
document.getElementById('holdDeg').addEventListener('keydown',
  e => { if(e.key === 'Enter') hold(); });
document.getElementById('btnHoldZero').addEventListener('click', () => send('H0'));
// Hold at wherever the platform currently sits, rounded to a whole degree.
document.getElementById('btnHoldHere').addEventListener('click',
  () => send('H' + Math.round(wrap(live.theta))));

const cap = () => {
  const v = document.getElementById('capDeg').value;
  if(v !== '') send('T' + v);
};
document.getElementById('btnCap').addEventListener('click', cap);
document.getElementById('capDeg').addEventListener('keydown',
  e => { if(e.key === 'Enter') cap(); });

const raw = () => {
  const el = document.getElementById('rawCmd');
  if(el.value.trim()){ send(el.value.trim()); el.value = ''; }
};
document.getElementById('btnRaw').addEventListener('click', raw);
document.getElementById('rawCmd').addEventListener('keydown',
  e => { if(e.key === 'Enter') raw(); });

// Space is a panic stop, unless a field has focus.
window.addEventListener('keydown', e => {
  if(e.code === 'Space' && !/INPUT|TEXTAREA/.test(document.activeElement.tagName)){
    e.preventDefault(); send('X');
  }
});

pushLog('Monitor ready. Zero the heading, then Hold at 0 to start the stream.', 't');
</script>
</body>
</html>
"""


class Link:
    """Serial port plus the parsed telemetry ring buffer."""

    def __init__(self, port, baud):
        self.port_name = port
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.logs = deque(maxlen=MAX_LOGS)
        self.seq = 0            # monotonic, so the browser can ask "since N"
        self.log_seq = 0
        self.connected = False
        self.t0 = time.time()

    # ---------- connection ----------

    def open(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.2)
            self.connected = True
            self._log(f"connected to {self.port_name} at {self.baud}")
        except Exception as exc:
            self.connected = False
            self._log(f"could not open {self.port_name}: {exc}")
            raise

    def _log(self, text):
        with self.lock:
            self.log_seq += 1
            self.logs.append({"n": self.log_seq,
                              "t": round(time.time() - self.t0, 2),
                              "text": text})

    # ---------- reader thread ----------

    def reader(self):
        buf = b""
        while True:
            if not self.connected:
                time.sleep(0.5)
                try:
                    self.open()
                except Exception:
                    continue
            try:
                chunk = self.ser.read(512)
            except Exception as exc:
                self._log(f"read error: {exc}")
                self.connected = False
                try:
                    self.ser.close()
                except Exception:
                    pass
                continue

            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._handle(raw.decode("utf-8", "replace").strip())

    def _handle(self, line):
        if not line:
            return
        parts = line.split(",")

        # HOLD telemetry: exactly six numeric fields.
        # theta_deg, target_deg, omega_p, omega_w, alpha, u
        if len(parts) == 6:
            try:
                v = [float(p) for p in parts]
            except ValueError:
                self._log(line)
                return
            with self.lock:
                self.seq += 1
                self.samples.append({
                    "n": self.seq,
                    "t": round(time.time() - self.t0, 3),
                    "theta": v[0], "target": v[1],
                    "wp": v[2], "ww": v[3],
                    "alpha": v[4], "u": v[5],
                })
            return

        # 7 fields = a T-capture CSV row or its header. Not live data.
        if len(parts) == 7:
            return

        # Everything else is firmware text: banners, STALL notices, G output.
        self._log(line)

    # ---------- writer ----------

    def send(self, cmd):
        if not self.connected or self.ser is None:
            self._log(f"not connected, dropped: {cmd}")
            return False
        try:
            self.ser.write((cmd + "\n").encode())
            self._log(f">> {cmd}")
            return True
        except Exception as exc:
            self._log(f"write error: {exc}")
            self.connected = False
            return False

    # ---------- snapshot for the browser ----------

    def snapshot(self, since, log_since):
        with self.lock:
            return {
                "connected": self.connected,
                "port": self.port_name,
                "seq": self.seq,
                "logSeq": self.log_seq,
                "samples": [s for s in self.samples if s["n"] > since],
                "logs": [g for g in self.logs if g["n"] > log_since],
            }


def pick_port(requested):
    if requested:
        return requested
    ports = list(list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Pair the HC-05 or plug in USB, then retry.")
    if len(ports) == 1:
        print(f"Using the only port available: {ports[0].device}")
        return ports[0].device
    print("Several ports available. Pick one with --port:")
    for p in ports:
        print(f"   {p.device:12s} {p.description}")
    sys.exit(1)


def make_handler(link):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass                                  # keep the console clean

        def _send(self, code, body, ctype):
            data = body if isinstance(body, bytes) else body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.startswith("/stream"):
                q = {}
                if "?" in self.path:
                    for kv in self.path.split("?", 1)[1].split("&"):
                        if "=" in kv:
                            k, v = kv.split("=", 1)
                            q[k] = v
                since = int(q.get("since", 0) or 0)
                logs = int(q.get("logs", 0) or 0)
                self._send(200, json.dumps(link.snapshot(since, logs)),
                           "application/json")
                return

            # Prefer an external live_monitor.html if one sits next to this
            # script AND has content -- that is the file to edit. A zero-byte or
            # missing file falls back to the copy embedded below, so the monitor
            # always renders something rather than a blank page.
            page = HERE / "live_monitor.html"
            body = PAGE
            if page.exists() and page.stat().st_size > 2000:
                body = page.read_text(encoding="utf-8")
            self._send(200, body, "text/html; charset=utf-8")

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            payload = self.rfile.read(length).decode("utf-8", "replace")
            try:
                cmd = json.loads(payload).get("cmd", "").strip()
            except Exception:
                cmd = ""
            ok = link.send(cmd) if cmd else False
            self._send(200, json.dumps({"ok": ok}), "application/json")

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="serial port, e.g. COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http", type=int, default=8000)
    args = ap.parse_args()

    link = Link(pick_port(args.port), args.baud)
    try:
        link.open()
    except Exception:
        print("Starting anyway; the reader will keep retrying the port.")

    threading.Thread(target=link.reader, daemon=True).start()

    server = ThreadingHTTPServer(("127.0.0.1", args.http), make_handler(link))
    print(f"\n  Live monitor:  http://localhost:{args.http}")
    print(f"  Serial:        {link.port_name} @ {link.baud}")
    ext = HERE / "live_monitor.html"
    if ext.exists() and ext.stat().st_size > 2000:
        print(f"  Page:          {ext.name} ({ext.stat().st_size} bytes)")
    elif ext.exists():
        print(f"  Page:          embedded  (ignoring {ext.name}, only "
              f"{ext.stat().st_size} bytes -- looks truncated)")
    else:
        print("  Page:          embedded")
    print("  Ctrl-C to stop.\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()