"""Replay + analytics UI for CS3AS episodes (npz or LeRobot v2.1).

Writes a self-contained HTML player (opens in any browser, including a
Hugging Face dataset repo) and an optional matplotlib GUI.
"""

from __future__ import annotations

import json
import webbrowser
from pathlib import Path
from typing import List, Optional, Sequence, Union

import numpy as np

from ..perception.skins import SKIN_PARTS
from .analytics import dataset_stats, episode_stats
from .dataset import Episode
from .lerobot_io import read_lerobot_episodes

HF_VIZ = "https://huggingface.co/spaces/lerobot/visualize_dataset"


def discover_episodes(path: Union[str, Path]) -> List[Episode]:
  path = Path(path)
  if path.is_file() and path.suffix == ".npz":
    return [Episode.load(path)]
  if not path.is_dir():
    raise FileNotFoundError(path)
  if (path / "meta" / "info.json").is_file():
    return read_lerobot_episodes(path)
  files = sorted(path.glob("*.npz")) + sorted(path.glob("episode_*.npz"))
  seen = []
  out = []
  for f in files:
    if f.resolve() in seen:
      continue
    seen.append(f.resolve())
    out.append(Episode.load(f))
  return out


def _payload(episode: Episode, max_points: int = 4000) -> dict:
  n = len(episode)
  step = max(1, n // max_points) if n > max_points else 1
  idx = np.arange(0, n, step, dtype=int)
  ts = episode.timestamps()
  if ts.size:
    t = (ts - ts[0]).astype(float)
  else:
    t = np.arange(n, dtype=float) / max(episode.fps, 1.0)
  joints = np.stack([f.joints for f in episode.frames]) if n else np.zeros((0, 6))
  pose = np.stack([f.pose for f in episode.frames]) if n else np.zeros((0, 6))
  return {
    "task": episode.task,
    "fps": float(episode.fps),
    "n_frames": n,
    "step": int(step),
    "t": t[idx].round(4).tolist(),
    "joints": joints[idx].round(3).tolist() if n else [],
    "pose": pose[idx, :3].round(2).tolist() if n else [],
    "gripper": np.asarray([f.gripper_position for f in episode.frames], dtype=float)[idx].round(1).tolist() if n else [],
    "prox": episode.skin_proximity()[idx].round(4).tolist() if n else [],
    "analog": episode.skin_analog()[idx].round(2).tolist() if n else [],
    "dist": episode.skin_distance()[idx].round(1).tolist() if n else [],
    "approach": episode.skin_approach()[idx].round(3).tolist() if n else [],
    "collision": [int(episode.frames[i].collision_state) for i in idx] if n else [],
    "stats": episode_stats(episode),
  }


def write_replay_html(
  episodes: Sequence[Episode],
  dest: Union[str, Path],
  *,
  title: str = "CS3AS episode replay",
  repo_id: Optional[str] = None,
) -> Path:
  dest = Path(dest)
  dest.parent.mkdir(parents=True, exist_ok=True)
  payload = {
    "title": title,
    "parts": list(SKIN_PARTS),
    "hf_viz": HF_VIZ,
    "repo_id": repo_id or "",
    "dataset": dataset_stats(episodes),
    "episodes": [_payload(ep) for ep in episodes],
  }
  blob = json.dumps(payload, separators=(",", ":")).replace("<", "\\u003c")
  dest.write_text(_HTML.replace("__PAYLOAD__", blob), encoding="utf-8")
  return dest


def open_replay(path: Union[str, Path]) -> None:
  webbrowser.open(Path(path).resolve().as_uri())


def replay_gui(episode: Episode):
  """Matplotlib play/scrub window. Requires a display + matplotlib."""
  try:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider
  except ImportError as exc:
    raise RuntimeError("pip install matplotlib") from exc

  n = len(episode)
  if n == 0:
    raise ValueError("empty episode")
  ts = episode.timestamps()
  t = ts - ts[0] if ts.size else np.arange(n) / max(episode.fps, 1.0)
  prox = episode.skin_proximity()
  analog = episode.skin_analog()
  joints = np.stack([f.joints for f in episode.frames])
  grip = np.asarray([f.gripper_position for f in episode.frames], dtype=float)
  stats = episode_stats(episode)

  fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
  fig.subplots_adjust(bottom=0.18, top=0.9)
  fig.canvas.manager.set_window_title(f"{episode.task}  {n} frames @ {episode.fps:.0f} Hz")
  for i in range(6):
    axes[0].plot(t, joints[:, i], lw=0.9, label=f"J{i+1}")
  axes[0].set_ylabel("joint deg")
  axes[0].legend(ncol=6, fontsize=7, loc="upper right")
  for i, name in enumerate(SKIN_PARTS):
    axes[1].plot(t, prox[:, i], lw=1.0, label=name)
    if analog.size:
      axes[1].plot(t, analog[:, i] / max(float(np.max(np.abs(analog))) or 1.0, 1.0), lw=0.6, alpha=0.4)
  axes[1].set_ylabel("proximity")
  axes[1].legend(ncol=4, fontsize=7, loc="upper right")
  axes[2].plot(t, grip, color="tab:purple", label="gripper")
  axes[2].set_ylabel("gripper")
  axes[2].set_xlabel("time s")
  vlines = [ax.axvline(t[0], color="k", lw=0.8, ls="--") for ax in axes]
  fig.suptitle(
    f"{episode.task}  {n} frames  {stats['duration_s']}s  "
    f"HIT {stats['hit_frames']} ({100 * stats['hit_rate']:.1f}%)"
  )

  ax_sl = fig.add_axes([0.12, 0.08, 0.62, 0.04])
  slider = Slider(ax_sl, "frame", 0, n - 1, valinit=0, valstep=1)
  playing = {"on": False}

  def seek(val):
    i = int(val)
    x = float(t[i])
    for line in vlines:
      line.set_xdata([x, x])
    fig.canvas.draw_idle()

  slider.on_changed(seek)

  def tick(_event):
    if not playing["on"]:
      return
    i = (int(slider.val) + 1) % n
    slider.set_val(i)

  timer = fig.canvas.new_timer(interval=int(1000 / max(episode.fps / 4, 8)))
  timer.add_callback(tick, None)
  timer.start()

  ax_play = fig.add_axes([0.78, 0.07, 0.08, 0.05])
  ax_stop = fig.add_axes([0.88, 0.07, 0.08, 0.05])
  b_play = Button(ax_play, "Play")
  b_stop = Button(ax_stop, "Pause")
  b_play.on_clicked(lambda _e: playing.__setitem__("on", True))
  b_stop.on_clicked(lambda _e: playing.__setitem__("on", False))
  plt.show()
  return fig


_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>CS3AS episode replay</title>
<style>
:root { --bg:#0f1419; --panel:#1a222c; --ink:#e7eef7; --muted:#8aa0b5; --acc:#3dbe8c; --hit:#e15b5b; --line:#2a3644; }
*{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 ui-sans-serif,system-ui,sans-serif}
header{padding:16px 20px;border-bottom:1px solid var(--line);display:flex;gap:16px;align-items:baseline;flex-wrap:wrap}
h1{font-size:18px;margin:0} .muted{color:var(--muted)}
main{display:grid;grid-template-columns:240px 1fr;min-height:calc(100vh - 58px)}
nav{border-right:1px solid var(--line);padding:12px;overflow:auto}
nav button{display:block;width:100%;text-align:left;background:transparent;color:var(--ink);border:0;padding:8px 10px;border-radius:6px;cursor:pointer}
nav button.active, nav button:hover{background:var(--panel)}
section{padding:16px 20px 88px}
.stats{display:flex;flex-wrap:wrap;gap:10px;margin-bottom:12px}
.chip{background:var(--panel);padding:8px 10px;border-radius:8px;min-width:110px}
.chip b{display:block;font-size:16px}
canvas{width:100%;height:160px;background:var(--panel);border-radius:8px;margin:8px 0}
.dock{position:fixed;bottom:0;left:240px;right:0;background:#121920;border-top:1px solid var(--line);padding:10px 20px;display:flex;gap:12px;align-items:center}
button.ctl{background:var(--acc);color:#062016;border:0;padding:8px 14px;border-radius:6px;font-weight:600;cursor:pointer}
input[type=range]{flex:1}
a{color:#7ec8ff}
@media (max-width:800px){main{grid-template-columns:1fr} nav{display:flex;overflow:auto} .dock{left:0}}
</style>
</head>
<body>
<header>
  <h1 id="title">CS3AS replay</h1>
  <span class="muted" id="summary"></span>
  <span class="muted">Hugging Face:
    <a id="hf" href="https://huggingface.co/spaces/lerobot/visualize_dataset" target="_blank">lerobot/visualize_dataset</a>
    (push this folder with <code>meta/info.json</code> + parquet)
  </span>
</header>
<main>
  <nav id="list"></nav>
  <section>
    <div class="stats" id="stats"></div>
    <div class="muted">SafeSkin proximity / analog (HIT frames in red)</div>
    <canvas id="c-skin" height="160"></canvas>
    <div class="muted">Joints</div>
    <canvas id="c-j" height="160"></canvas>
    <div class="muted">Gripper 0–1000 + XYZ</div>
    <canvas id="c-g" height="160"></canvas>
  </section>
</main>
<div class="dock">
  <button class="ctl" id="play">Play</button>
  <input id="scrub" type="range" min="0" max="0" value="0"/>
  <span id="clock" class="muted">0.00 s</span>
</div>
<script>
const DATA = __PAYLOAD__;
const parts = DATA.parts;
document.getElementById('title').textContent = DATA.title;
const ds = DATA.dataset;
document.getElementById('summary').textContent =
  ds.episodes + ' ep · ' + ds.n_frames + ' frames · ' + ds.duration_s + 's · HIT ' + (100*ds.hit_rate).toFixed(1) + '%';
if (DATA.repo_id) {
  document.getElementById('hf').href = DATA.hf_viz + '?path=' + encodeURIComponent(DATA.repo_id) + '/0';
}
const list = document.getElementById('list');
let cur = 0, frame = 0, playing = false, last = 0;
DATA.episodes.forEach((ep,i) => {
  const b = document.createElement('button');
  b.textContent = (i+1) + '. ' + ep.task + '  n=' + ep.n_frames;
  b.onclick = () => select(i);
  list.appendChild(b);
});
function select(i){
  cur = i; frame = 0; playing = false;
  [...list.children].forEach((el,k)=>el.classList.toggle('active', k===i));
  const ep = DATA.episodes[i], s = ep.stats;
  document.getElementById('stats').innerHTML = [
    ['frames', s.n_frames], ['fps', s.fps], ['dur s', s.duration_s],
    ['HIT', s.hit_frames + ' / ' + (100*s.hit_rate).toFixed(1) + '%'],
    ['collision', s.collision_frames], ['cam', s.camera_frames]
  ].map(([k,v]) => '<div class="chip"><span class="muted">'+k+'</span><b>'+v+'</b></div>').join('');
  document.getElementById('scrub').max = Math.max(0, ep.t.length-1);
  document.getElementById('scrub').value = 0;
  draw();
}
function series(canvas, t, ys, colors, hit){
  const ctx = canvas.getContext('2d');
  const w = canvas.width = canvas.clientWidth, h = canvas.height = canvas.height;
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle = '#1a222c'; ctx.fillRect(0,0,w,h);
  if (!t.length) return;
  let lo=Infinity, hi=-Infinity;
  ys.forEach(y => y.forEach(v => { if(v<lo)lo=v; if(v>hi)hi=v; }));
  if (!(hi>lo)) { lo-=1; hi+=1; }
  const x = i => i/(t.length-1||1)*w;
  const Y = v => h-8 - (v-lo)/(hi-lo)*(h-16);
  if (hit){
    ctx.fillStyle = 'rgba(225,91,91,0.18)';
    hit.forEach((v,i)=>{ if(Math.abs(v)>0) ctx.fillRect(x(i)-1,0,3,h); });
  }
  ys.forEach((y,k)=>{
    ctx.beginPath(); ctx.strokeStyle = colors[k%colors.length]; ctx.lineWidth=1.4;
    y.forEach((v,i)=>{ const X=x(i), yy=Y(v); i?ctx.lineTo(X,yy):ctx.moveTo(X,yy); });
    ctx.stroke();
  });
  const ep = DATA.episodes[cur];
  const xf = x(frame);
  ctx.strokeStyle='#e7eef7'; ctx.setLineDash([4,3]);
  ctx.beginPath(); ctx.moveTo(xf,0); ctx.lineTo(xf,h); ctx.stroke(); ctx.setLineDash([]);
}
function draw(){
  const ep = DATA.episodes[cur];
  if (!ep) return;
  const hit = (ep.approach||[]).map(r => r.some(v => Math.abs(v)>0)?1:0);
  const prox = parts.map((_,k)=> (ep.prox||[]).map(r=>r[k]||0));
  const analog = parts.map((_,k)=> (ep.analog||[]).map(r=>r[k]||0));
  series(document.getElementById('c-skin'), ep.t, prox.concat(analog),
    ['#3dbe8c','#5b9cf6','#e0b044','#d67ad3','#1f7a55','#355d96','#9a7420','#7a3d78'], hit);
  const js = [0,1,2,3,4,5].map(k => (ep.joints||[]).map(r=>r[k]||0));
  series(document.getElementById('c-j'), ep.t, js, ['#7ec8ff','#3dbe8c','#e0b044','#d67ad3','#e15b5b','#a0b4c8']);
  const g = [(ep.gripper||[])];
  const xyz = [0,1,2].map(k => (ep.pose||[]).map(r=>r[k]||0));
  series(document.getElementById('c-g'), ep.t, g.concat(xyz), ['#d67ad3','#7ec8ff','#3dbe8c','#e0b044']);
  document.getElementById('clock').textContent = (ep.t[frame]||0).toFixed(3)+' s  #'+frame;
}
document.getElementById('play').onclick = () => {
  playing = !playing;
  document.getElementById('play').textContent = playing ? 'Pause' : 'Play';
  last = performance.now();
};
document.getElementById('scrub').oninput = e => { frame = +e.target.value; draw(); };
function loop(now){
  if (playing) {
    const ep = DATA.episodes[cur];
    const dt = (now-last)/1000;
    const step = Math.max(1, Math.round(dt * (ep.fps||30) / (ep.step||1)));
    if (step>=1){ frame = (frame+step) % ep.t.length; last = now; document.getElementById('scrub').value=frame; draw(); }
  }
  requestAnimationFrame(loop);
}
select(0); requestAnimationFrame(loop);
</script>
</body>
</html>
"""
