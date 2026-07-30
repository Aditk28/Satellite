#!/usr/bin/env python3
"""
make_replay.py

Builds a standalone HTML replay of a filtered calibration run: a top-down
view of the platform with the reaction wheel inside it, both turning from
the recorded data, alongside the telemetry traces and a momentum-exchange
readout. Run filter_calibration.py first -- this reads the filtered/ folder
so the gyro is already de-biased and sign-consistent with the wheel.

The output is one self-contained .html file with the data embedded. No
server, no dependencies, no network -- open it in a browser, or drop it in
the repo for anyone reading the project to scrub through.

WHAT IT SHOWS:
  - Outer disc: the platform, turning by the integrated (bias-corrected)
    gyro heading. Its notch marks the docking face.
  - Inner disc: the reaction wheel, turning by the integrated encoder
    angle. At high speed the spokes alias, exactly as a real strobed wheel
    would -- read the numeric velocity, not the spokes.
  - The two turn in opposition, which is the whole point of the machine:
    the wheel takes angular momentum, the platform takes the equal and
    opposite share.
  - Momentum bar: relative angular momentum in each body, in the units the
    data actually supports (wheel rad/s and platform dps are NOT scaled to
    a common inertia -- J_w and J_p aren't measured yet, so this bar shows
    the exchange qualitatively, not a conserved total).

USAGE:
    python make_replay.py calibration_run_20260729_112929/filtered
    python make_replay.py <filtered_folder> --out replay.html --frames 500
"""

import argparse
import glob
import json
import os
import re
import sys

try:
    import numpy as np
    import pandas as pd
except ImportError:
    print("This script needs numpy and pandas:  pip install numpy pandas")
    sys.exit(1)

RE_TITLE = re.compile(r"^#\s*test\s+(\d+)/(\d+):\s*(.*)$")
RE_META = re.compile(r"^#\s*from=([-\d.]+)V\s+to=([-\d.]+)V")


def read_filtered(path):
    meta = {}
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            m = RE_TITLE.match(line)
            if m:
                meta["num"], meta["label"] = int(m.group(1)), m.group(3)
            m = RE_META.match(line)
            if m:
                meta["from_v"], meta["to_v"] = float(m.group(1)), float(m.group(2))
    df = pd.read_csv(path, comment="#")
    return df, meta


def downsample(df, n):
    if len(df) <= n:
        return df.reset_index(drop=True)
    idx = np.linspace(0, len(df) - 1, n).astype(int)
    return df.iloc[idx].reset_index(drop=True)


def build_payload(folder, frames):
    files = sorted(glob.glob(os.path.join(folder, "test*.csv")))
    if not files:
        return None
    out = []
    for path in files:
        df, meta = read_filtered(path)
        d = downsample(df, frames)
        step_rows = d.index[d["phase"] == "B"].tolist()
        out.append({
            "num": meta.get("num", 0),
            "label": meta.get("label", os.path.basename(path)),
            "fromV": meta.get("from_v", 0.0),
            "toV": meta.get("to_v", 0.0),
            "stepIdx": int(step_rows[0]) if step_rows else 0,
            "t": [round(float(x), 4) for x in d["t_s"]],
            "cmd": [round(float(x), 3) for x in d["targetV"]],
            "wv": [round(float(x), 3) for x in d["wheel_vel"]],
            "wa": [round(float(x), 3) for x in d["wheel_angle_rad"]],
            "pr": [round(float(x), 3) for x in d["gyro_dps"]],
            "pd": [round(float(x), 3) for x in d["platform_deg"]],
        })
    out.sort(key=lambda x: x["num"])
    return out


HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Reaction wheel attitude replay</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@400;600&family=Barlow:wght@400;500&family=IBM+Plex+Mono:wght@400;500&display=swap" rel="stylesheet">
<style>
  :root{
    --bg:#0E141A; --panel:#161F28; --panel2:#1B2732; --rule:#253240;
    --wheel:#E0932F; --platform:#4FB8D9; --heading:#9B7FD4; --cmd:#8497A6;
    --ink:#CBD8E2; --muted:#74899A; --hot:#D2603F;
    --mono:'IBM Plex Mono',ui-monospace,monospace;
    --disp:'Barlow Condensed','Barlow',system-ui,sans-serif;
    --body:'Barlow',system-ui,sans-serif;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);font-family:var(--body);
       -webkit-font-smoothing:antialiased}
  .wrap{max-width:1180px;margin:0 auto;padding:26px 20px 40px}
  header{border-bottom:1px solid var(--rule);padding-bottom:14px;margin-bottom:20px}
  .eyebrow{font-family:var(--mono);font-size:11px;letter-spacing:.16em;
           text-transform:uppercase;color:var(--muted)}
  h1{font-family:var(--disp);font-weight:600;font-size:31px;letter-spacing:.01em;
     margin:4px 0 0;line-height:1.1}
  h1 span{color:var(--muted);font-weight:400}
  .grid{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:18px}
  @media(max-width:860px){.grid{grid-template-columns:1fr}}
  .card{background:var(--panel);border:1px solid var(--rule);border-radius:3px;padding:16px}
  .card h2{font-family:var(--mono);font-size:10.5px;letter-spacing:.15em;
           text-transform:uppercase;color:var(--muted);margin:0 0 12px;font-weight:500}
  svg{display:block;width:100%;height:auto}
  .readout{display:flex;justify-content:space-between;align-items:baseline;
           padding:9px 0;border-bottom:1px solid var(--rule)}
  .readout:last-child{border-bottom:none}
  .readout .k{font-family:var(--mono);font-size:10.5px;letter-spacing:.09em;
              text-transform:uppercase;color:var(--muted)}
  .readout .v{font-family:var(--mono);font-size:16px;font-variant-numeric:tabular-nums}
  .v.wheel{color:var(--wheel)} .v.platform{color:var(--platform)}
  .v.heading{color:var(--heading)} .v.cmd{color:var(--ink)}
  .mombar{height:9px;background:var(--panel2);border-radius:2px;position:relative;
          overflow:hidden;margin-top:6px}
  .mombar i{position:absolute;top:0;bottom:0;left:50%;transform-origin:left center}
  .mombar .w{background:var(--wheel)} .mombar .p{background:var(--platform)}
  .momlabel{display:flex;justify-content:space-between;font-family:var(--mono);
            font-size:9.5px;letter-spacing:.08em;color:var(--muted);margin-top:5px}
  .controls{display:flex;gap:12px;align-items:center;margin-top:16px;flex-wrap:wrap}
  button{font-family:var(--mono);font-size:12px;letter-spacing:.08em;background:var(--panel2);
         color:var(--ink);border:1px solid var(--rule);border-radius:2px;padding:8px 15px;
         cursor:pointer;text-transform:uppercase}
  button:hover{border-color:var(--muted)}
  button:focus-visible{outline:2px solid var(--platform);outline-offset:2px}
  select{font-family:var(--mono);font-size:12px;background:var(--panel2);color:var(--ink);
         border:1px solid var(--rule);border-radius:2px;padding:8px 10px;max-width:100%}
  select:focus-visible{outline:2px solid var(--platform);outline-offset:2px}
  input[type=range]{flex:1;min-width:180px;accent-color:var(--platform)}
  .foot{font-family:var(--mono);font-size:10.5px;color:var(--muted);margin-top:22px;
        line-height:1.75;border-top:1px solid var(--rule);padding-top:14px}
  @media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div class="eyebrow">Reaction wheel testbed &middot; recorded telemetry replay</div>
    <h1 id="title">Test — <span id="subtitle"></span></h1>
  </header>

  <div class="grid">
    <div class="card">
      <h2>Top-down attitude</h2>
      <svg id="attitude" viewBox="0 0 460 380" role="img" aria-label="Top-down view of platform and reaction wheel">
        <defs>
          <marker id="arw" markerWidth="7" markerHeight="7" refX="5.5" refY="3.5" orient="auto">
            <path d="M0,0 L7,3.5 L0,7 z" fill="#4FB8D9"/>
          </marker>
        </defs>
        <g id="grid"></g>
        <g id="platformG">
          <circle cx="230" cy="180" r="140" fill="#111A22" stroke="#4FB8D9" stroke-width="1.6" opacity="0.95"/>
          <path d="M230 40 L214 62 L246 62 Z" fill="#4FB8D9"/>
          <line x1="230" y1="180" x2="230" y2="52" stroke="#4FB8D9" stroke-width="1.1" opacity="0.55"/>
          <circle cx="230" cy="180" r="140" fill="none" stroke="#4FB8D9" stroke-width="7" opacity="0.07"/>
        </g>
        <g id="wheelG">
          <circle cx="230" cy="180" r="62" fill="#1A1409" stroke="#E0932F" stroke-width="1.5"/>
          <g id="spokes"></g>
          <circle cx="230" cy="180" r="9" fill="#E0932F" opacity="0.85"/>
        </g>
        <circle cx="230" cy="180" r="2.5" fill="#CBD8E2"/>
        <text x="230" y="352" text-anchor="middle" fill="#74899A"
              font-family="IBM Plex Mono, monospace" font-size="10.5" letter-spacing="1.4">
          PLATFORM HEADING <tspan id="hdgTxt" fill="#4FB8D9">0.0</tspan>&#176;
        </text>
      </svg>
    </div>

    <div class="card">
      <h2>Telemetry</h2>
      <div class="readout"><span class="k">t</span><span class="v cmd" id="rTime">0.000 s</span></div>
      <div class="readout"><span class="k">command</span><span class="v cmd" id="rCmd">0.00 V</span></div>
      <div class="readout"><span class="k">wheel &omega;</span><span class="v wheel" id="rWv">0.00 rad/s</span></div>
      <div class="readout"><span class="k">platform rate</span><span class="v platform" id="rPr">0.00 dps</span></div>
      <div class="readout"><span class="k">heading</span><span class="v heading" id="rPd">0.00&#176;</span></div>

      <h2 style="margin-top:18px">Momentum exchange</h2>
      <div class="mombar"><i class="w" id="barW"></i></div>
      <div class="mombar"><i class="p" id="barP"></i></div>
      <div class="momlabel"><span>WHEEL</span><span>PLATFORM</span></div>
    </div>
  </div>

  <div class="card" style="margin-top:18px">
    <h2>Traces</h2>
    <svg id="trace" viewBox="0 0 1120 260" role="img" aria-label="Time series of wheel velocity and platform rate"></svg>
    <div class="controls">
      <button id="play">Play</button>
      <input type="range" id="scrub" min="0" max="100" value="0" step="1" aria-label="Scrub through time">
      <select id="speed" aria-label="Playback speed">
        <option value="0.25">0.25&times;</option>
        <option value="0.5">0.5&times;</option>
        <option value="1" selected>1&times;</option>
        <option value="2">2&times;</option>
      </select>
      <select id="testSel" aria-label="Select test"></select>
    </div>
  </div>

  <div class="foot">
    Wheel and platform turn in opposition &mdash; the wheel takes angular momentum, the platform takes the equal and opposite share.<br>
    Spokes alias at speed, like a strobed wheel; read the numeric velocity instead. Momentum bars are relative within each body,
    not a conserved total &mdash; J<sub>w</sub> and J<sub>p</sub> are not measured yet, so the two channels cannot share units.<br>
    Gyro bias removed and sign aligned to the wheel convention by filter_calibration.py.
  </div>
</div>

<script>
const DATA = __DATA__;
let cur = 0, idx = 0, playing = false, last = 0, speed = 1;

const $ = id => document.getElementById(id);
const platformG = $('platformG'), wheelG = $('wheelG'), spokes = $('spokes');

// Wheel spokes
let s = '';
for (let i = 0; i < 6; i++) {
  const a = i * Math.PI / 3;
  s += `<line x1="${230 + 11*Math.cos(a)}" y1="${180 + 11*Math.sin(a)}" x2="${230 + 58*Math.cos(a)}" y2="${180 + 58*Math.sin(a)}" stroke="#E0932F" stroke-width="2.4" opacity="0.75"/>`;
}
spokes.innerHTML = s;

// Static polar grid behind the platform
let g = '';
for (let r = 40; r <= 160; r += 40) g += `<circle cx="230" cy="180" r="${r}" fill="none" stroke="#253240" stroke-width="0.8"/>`;
for (let i = 0; i < 12; i++) {
  const a = i * Math.PI / 6;
  g += `<line x1="${230 + 150*Math.cos(a)}" y1="${180 + 150*Math.sin(a)}" x2="${230 + 165*Math.cos(a)}" y2="${180 + 165*Math.sin(a)}" stroke="#253240" stroke-width="0.9"/>`;
}
$('grid').innerHTML = g;

// Test selector
const sel = $('testSel');
DATA.forEach((d, i) => {
  const o = document.createElement('option');
  o.value = i; o.textContent = `${String(d.num).padStart(2,'0')} — ${d.label}`;
  sel.appendChild(o);
});

function extent(arr){ let a=Infinity,b=-Infinity; for(const v of arr){if(v<a)a=v;if(v>b)b=v;} if(a===b){a-=1;b+=1;} return [a,b]; }

function drawTrace(){
  const d = DATA[cur];
  const W=1120,H=260,L=58,R=16,T=14,B=30;
  const iw=W-L-R, ih=H-T-B;
  const t0=d.t[0], t1=d.t[d.t.length-1];
  const [wa,wb]=extent(d.wv), [pa,pb]=extent(d.pr);
  const X = v => L + (v-t0)/(t1-t0||1)*iw;
  const Yw = v => T + ih - (v-wa)/((wb-wa)||1)*ih;
  const Yp = v => T + ih - (v-pa)/((pb-pa)||1)*ih;

  let h='';
  for(let i=0;i<=4;i++){ const y=T+ih*i/4; h+=`<line x1="${L}" y1="${y}" x2="${W-R}" y2="${y}" stroke="#253240" stroke-width="0.8"/>`; }
  // step marker
  const xs = X(d.t[d.stepIdx]);
  h+=`<line x1="${xs}" y1="${T}" x2="${xs}" y2="${T+ih}" stroke="#D2603F" stroke-width="1.3" opacity="0.85"/>`;
  h+=`<text x="${xs+5}" y="${T+12}" fill="#D2603F" font-family="IBM Plex Mono, monospace" font-size="10">STEP &#8594; ${d.toV}V</text>`;

  let pw='', pp='';
  for(let i=0;i<d.t.length;i++){
    pw += (i?'L':'M')+X(d.t[i]).toFixed(1)+' '+Yw(d.wv[i]).toFixed(1);
    pp += (i?'L':'M')+X(d.t[i]).toFixed(1)+' '+Yp(d.pr[i]).toFixed(1);
  }
  h+=`<path d="${pw}" fill="none" stroke="#E0932F" stroke-width="1.7"/>`;
  h+=`<path d="${pp}" fill="none" stroke="#4FB8D9" stroke-width="1.5" opacity="0.9"/>`;
  h+=`<line id="ph" x1="${L}" y1="${T}" x2="${L}" y2="${T+ih}" stroke="#CBD8E2" stroke-width="1.2" opacity="0.8"/>`;
  h+=`<text x="${L}" y="${H-9}" fill="#74899A" font-family="IBM Plex Mono, monospace" font-size="10">${t0.toFixed(2)}s</text>`;
  h+=`<text x="${W-R}" y="${H-9}" text-anchor="end" fill="#74899A" font-family="IBM Plex Mono, monospace" font-size="10">${t1.toFixed(2)}s</text>`;
  h+=`<text x="${L}" y="${H-9}" dx="70" fill="#E0932F" font-family="IBM Plex Mono, monospace" font-size="10">&#9473; wheel &omega; (rad/s)</text>`;
  h+=`<text x="${L}" y="${H-9}" dx="215" fill="#4FB8D9" font-family="IBM Plex Mono, monospace" font-size="10">&#9473; platform rate (dps)</text>`;
  $('trace').innerHTML = h;
  window._X = X; window._T = T; window._ih = ih;
}

function render(){
  const d = DATA[cur];
  // idx advances fractionally during playback and scrubbing -- round it
  // before indexing, or array lookups land on undefined.
  const i = Math.max(0, Math.min(Math.round(idx), d.t.length-1));
  platformG.setAttribute('transform', `rotate(${d.pd[i]} 230 180)`);
  wheelG.setAttribute('transform', `rotate(${(d.wa[i]*180/Math.PI)%360} 230 180)`);
  $('hdgTxt').textContent = d.pd[i].toFixed(1);
  $('rTime').textContent = d.t[i].toFixed(3)+' s';
  $('rCmd').textContent  = d.cmd[i].toFixed(2)+' V';
  $('rWv').textContent   = d.wv[i].toFixed(2)+' rad/s';
  $('rPr').textContent   = d.pr[i].toFixed(2)+' dps';
  $('rPd').textContent   = d.pd[i].toFixed(2)+'\u00B0';

  const mw = Math.max(...d.wv.map(Math.abs))||1, mp = Math.max(...d.pr.map(Math.abs))||1;
  const bw = $('barW'), bp = $('barP');
  bw.style.width = Math.abs(d.wv[i])/mw*50+'%';
  bw.style.transform = d.wv[i]<0 ? 'scaleX(-1)' : 'scaleX(1)';
  bp.style.width = Math.abs(d.pr[i])/mp*50+'%';
  bp.style.transform = d.pr[i]<0 ? 'scaleX(-1)' : 'scaleX(1)';

  const ph = document.getElementById('ph');
  if (ph && window._X){ const x = window._X(d.t[i]); ph.setAttribute('x1',x); ph.setAttribute('x2',x); }
  $('scrub').value = i/(d.t.length-1)*100;
}

function loadTest(k){
  cur = k; idx = 0; playing = false; $('play').textContent='Play';
  const d = DATA[cur];
  $('title').innerHTML = `Test ${String(d.num).padStart(2,'0')} — <span>${d.label}</span>`;
  drawTrace(); render();
}

function tick(ts){
  if(playing){
    const d = DATA[cur];
    if(!last) last = ts;
    const dt = (ts-last)/1000*speed; last = ts;
    const span = d.t[d.t.length-1]-d.t[0];
    const frac = dt/ (span||1);
    idx += frac*(d.t.length-1);
    if(idx >= d.t.length-1){ idx = d.t.length-1; playing=false; $('play').textContent='Replay'; }
    render();
  } else last = 0;
  requestAnimationFrame(tick);
}

$('play').onclick = () => {
  const d = DATA[cur];
  if(idx >= d.t.length-1) idx = 0;
  playing = !playing; last = 0;
  $('play').textContent = playing ? 'Pause' : 'Play';
};
$('scrub').oninput = e => { const d=DATA[cur]; idx = e.target.value/100*(d.t.length-1); playing=false; $('play').textContent='Play'; render(); };
$('speed').onchange = e => speed = parseFloat(e.target.value);
sel.onchange = e => loadTest(parseInt(e.target.value));

loadTest(0);
requestAnimationFrame(tick);
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser(description="Build a standalone HTML attitude replay.")
    ap.add_argument("filtered_folder", help="The filtered/ folder from filter_calibration.py")
    ap.add_argument("--out", default=None, help="Output HTML path (default: <filtered_folder>/replay.html)")
    ap.add_argument("--frames", type=int, default=500, help="Max frames per test (default 500)")
    args = ap.parse_args()

    payload = build_payload(args.filtered_folder, args.frames)
    if payload is None:
        print(f"No test*.csv found in {args.filtered_folder}")
        print("Run filter_calibration.py first, then point this at the filtered/ folder.")
        sys.exit(1)

    out = args.out or os.path.join(args.filtered_folder, "replay.html")
    html = HTML.replace("__DATA__", json.dumps(payload, separators=(",", ":")))
    with open(out, "w", encoding="utf-8") as f:
        f.write(html)

    kb = os.path.getsize(out) / 1024
    print(f"  wrote {out}  ({len(payload)} tests, {kb:.0f} KB)")
    print("  open it in any browser -- no server needed")


if __name__ == "__main__":
    main()