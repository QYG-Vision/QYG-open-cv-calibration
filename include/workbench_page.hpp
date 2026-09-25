#pragma once

namespace qd {

// 标定工作台前端页面 (内嵌单页应用)
// 由 calibrationWorkbench 在启动时通过 WebViewer::setCustomPage() 注入

static const char* WORKBENCH_PAGE = R"html(<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>标定工作台</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1e1e2e;color:#cdd6f4;font-family:system-ui,-apple-system,sans-serif;
     display:flex;flex-direction:column;height:100vh;overflow:hidden}
header{background:#181825;padding:10px 20px;display:flex;align-items:center;
       justify-content:space-between;border-bottom:1px solid #313244;min-height:48px}
header h1{font-size:16px;font-weight:600}
header .dot{width:10px;height:10px;border-radius:50%;background:#a6e3a1;
            display:inline-block;margin-right:8px}
header .dot.off{background:#f38ba8}
header .info{font-size:12px;color:#a6adc8;display:flex;gap:20px}
header .info span{color:#6c7086}header .info strong{color:#cdd6f4}

#app{display:flex;flex:1;overflow:hidden}

/* ---- left panel ---- */
#left{width:220px;min-width:180px;background:#181825;border-right:1px solid #313244;
      display:flex;flex-direction:column;overflow-y:auto}
#left h3{padding:12px 16px 6px;font-size:12px;color:#6c7086;text-transform:uppercase;
         letter-spacing:1px}
.task-card{background:#1e1e2e;margin:4px 12px;padding:10px 12px;border-radius:8px;
           cursor:pointer;border:2px solid transparent;transition:border .15s}
.task-card:hover{border-color:#45475a}
.task-card.active{border-color:#89b4fa}
.task-card .name{font-size:13px;font-weight:500}
.task-card .desc{font-size:11px;color:#6c7086;margin-top:2px}
.task-card .status{font-size:10px;margin-top:4px;display:inline-block;padding:2px 6px;
                  border-radius:10px}
.status-ready{background:#a6e3a1;color:#1e1e2e}
.status-running{background:#89b4fa;color:#1e1e2e}
.status-done{background:#f9e2af;color:#1e1e2e}
.status-err{background:#f38ba8;color:#1e1e2e}

#left .actions{padding:8px 12px;display:flex;flex-direction:column;gap:4px}
#left .actions button{width:100%;padding:8px;border:none;border-radius:6px;
    font-size:12px;font-weight:600;cursor:pointer}
.btn-start{background:#a6e3a1;color:#1e1e2e}
.btn-stop{background:#f38ba8;color:#1e1e2e}
.btn-recal{background:#f9e2af;color:#1e1e2e}
.btn-start:disabled,.btn-stop:disabled,.btn-recal:disabled{opacity:.4;cursor:not-allowed}

/* ---- center ---- */
#center{flex:1;display:flex;flex-direction:column;min-width:0;overflow:hidden}
#video-area{flex:1;display:flex;flex-direction:column;align-items:center;
            justify-content:center;position:relative;background:#11111b;min-height:300px}
#video-area img{max-width:100%;max-height:100%;object-fit:contain}
#video-area .placeholder{color:#6c7086;font-size:14px}
.stream-overlay{position:absolute;top:16px;left:12px;font-size:13px;color:#a6e3a1;
                background:rgba(0,0,0,.6);padding:3px 8px;border-radius:4px}

#action-bar{display:flex;gap:6px;padding:8px 12px;background:#181825;border-top:1px solid #313244;
            flex-wrap:wrap;align-items:center}
#action-bar button{padding:6px 14px;border:none;border-radius:6px;
    font-size:12px;font-weight:600;cursor:pointer;background:#45475a;color:#cdd6f4}
#action-bar button:hover{background:#585b70}
#action-bar button:disabled{opacity:.4;cursor:not-allowed}
#action-bar button.primary{background:#89b4fa;color:#1e1e2e}
#action-bar button.warn{background:#f38ba8;color:#1e1e2e}
#action-bar .sep{width:1px;height:20px;background:#313244;margin:0 6px}
#action-bar .hint{font-size:11px;color:#6c7086;margin-left:auto}

/* ---- right panel ---- */
#right{width:320px;min-width:280px;background:#181825;border-left:1px solid #313244;
       display:flex;flex-direction:column;overflow-y:auto}
#right h3{padding:12px 16px 8px;font-size:12px;color:#6c7086;text-transform:uppercase;
          letter-spacing:1px}
#right label{font-size:11px;color:#a6adc8;display:block;margin:4px 16px 2px}
#right input,#right select{width:calc(100% - 32px);margin:0 16px 6px;padding:6px 8px;
    background:#313244;color:#cdd6f4;border:1px solid #45475a;border-radius:4px;font-size:12px}
#right input:focus,#right select:focus{outline:none;border-color:#89b4fa}
#right .cfg-save{margin:8px 16px;padding:8px;background:#a6e3a1;color:#1e1e2e;border:none;
    border-radius:6px;font-weight:600;cursor:pointer;width:calc(100% - 32px)}
#right .cfg-save:disabled{opacity:.4}

#log-panel{flex:1;min-height:120px;background:#11111b;margin:4px 8px 8px;border-radius:6px;
           overflow-y:auto;padding:6px 10px;font-size:11px;font-family:monospace;max-height:200px}
#log-panel .line{color:#a6adc8;padding:1px 0;white-space:pre-wrap;word-break:break-all}
#log-panel .line.warn{color:#f9e2af}
#log-panel .line.err{color:#f38ba8}

.result-block{margin:4px 16px;padding:8px 10px;background:#1e1e2e;border-radius:6px;font-size:11px}
.result-block .label{color:#6c7086}
.result-block .value{color:#a6e3a1;font-weight:500}

/* ---- review modal ---- */
.modal-overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.7);z-index:200;
               align-items:center;justify-content:center}
.modal-overlay.show{display:flex}
.modal{background:#1e1e2e;border:1px solid #45475a;border-radius:12px;padding:24px;
       max-width:600px;width:90%;max-height:80vh;overflow-y:auto}
.modal h2{font-size:16px;margin-bottom:12px;color:#f9e2af}
.modal .compare{display:flex;gap:16px;margin:12px 0}
.modal .compare>div{flex:1}
.modal .compare h4{font-size:12px;color:#6c7086;margin-bottom:4px}
.modal .compare pre{background:#11111b;padding:8px;border-radius:4px;font-size:10px;
    overflow-x:auto;color:#a6adc8;white-space:pre-wrap}
.modal .actions{display:flex;gap:8px;margin-top:16px;justify-content:flex-end}
.modal .actions button{padding:8px 20px;border:none;border-radius:6px;
    font-weight:600;cursor:pointer;font-size:13px}
.btn-accept{background:#a6e3a1;color:#1e1e2e}
.btn-reject{background:#f38ba8;color:#1e1e2e}

.toast{position:fixed;bottom:60px;left:50%;transform:translateX(-50%);z-index:300;
       background:#b4befe;color:#1e1e2e;padding:6px 18px;border-radius:20px;
       font-size:13px;font-weight:600;opacity:0;transition:opacity .2s;pointer-events:none}
.toast.show{opacity:1}
</style>
</head>
<body tabindex="0">

<header>
  <h1><span class="dot" id="dot"></span>标定工作台</h1>
  <div class="info">
    <span>设备</span><strong id="hdr-device">--</strong>
    <span>阶段</span><strong id="hdr-phase">空闲</strong>
    <span>串口</span><strong id="hdr-serial">--</strong>
    <span>运行</span><strong id="hdr-uptime">0s</strong>
  </div>
</header>

<div id="app">
<!-- left panel -->
<div id="left">
  <h3>任务列表</h3>
  <div id="task-list"></div>
  <div class="actions">
    <button class="btn-start" id="btn-start" onclick="startTask()">&#9654; 开始任务</button>
    <button class="btn-stop" id="btn-stop" onclick="stopTask()" disabled>&#9632; 停止任务</button>
    <button class="btn-recal" id="btn-recal" onclick="recalTask()" disabled>重新标定</button>
  </div>
</div>

<!-- center -->
<div id="center">
  <div id="video-area">
    <div class="placeholder" id="video-placeholder">等待视频流连接...</div>
    <img id="stream-img" src="" alt="video" style="display:none">
    <div class="stream-overlay" id="stream-hud" style="display:none"></div>
  </div>
  <div id="action-bar">
    <button id="act-collect" onclick="sendAction('collect')" disabled>采集</button>
    <button id="act-auto" onclick="sendAction('toggle_auto_collect')" disabled>自动采集</button>
    <button id="act-skip" onclick="sendAction('skip')" disabled>跳过</button>
    <span class="sep"></span>
    <button class="primary" id="act-compute" onclick="sendAction('compute')" disabled>开始计算</button>
    <span class="sep"></span>
    <button id="act-reset" onclick="sendAction('reset')" disabled>重置统计</button>
    <span class="hint" id="hint-text">选择一个任务开始</span>
  </div>
</div>

<!-- right panel -->
<div id="right">
  <h3>配置文件</h3>
  <label>设备类型</label>
  <select id="cfg-device" onchange="cfgChanged()">
    <option value="HIK">HIK</option><option value="UVC">UVC</option><option value="IMG">IMG</option>
  </select>
  <label>标定板类型</label>
  <select id="cfg-pattern" onchange="cfgChanged()">
    <option value="chessboard">棋盘格</option><option value="circles">对称圆</option><option value="acircles">非对称圆</option>
  </select>
  <label>内角点数 (行)</label><input id="cfg-rows" type="number" onchange="cfgChanged()">
  <label>内角点数 (列)</label><input id="cfg-cols" type="number" onchange="cfgChanged()">
  <label>方格边长 (mm)</label><input id="cfg-square" type="number" step="0.1" onchange="cfgChanged()">
  <label>采集图像保存路径</label><input id="cfg-cam-save" onchange="cfgChanged()">
  <label>手眼数据保存路径</label><input id="cfg-he-save" onchange="cfgChanged()">

  <details style="margin:0 16px">
    <summary style="font-size:12px;color:#89b4fa;cursor:pointer">高级配置</summary>
    <label>串口设备</label><input id="cfg-serial-port" onchange="cfgChanged()">
    <label>串口波特率</label><input id="cfg-serial-baud" type="number" onchange="cfgChanged()">
    <label>HIK 曝光 (us)</label><input id="cfg-hik-exp" type="number" onchange="cfgChanged()">
    <label>HIK 增益</label><input id="cfg-hik-gain" type="number" step="0.1" onchange="cfgChanged()">
    <label>IMG 路径</label><input id="cfg-img-path" onchange="cfgChanged()">
    <label>自动采集间隔 (ms)</label><input id="cfg-ac-int" type="number" onchange="cfgChanged()">
    <label>自动采集距离阈值</label><input id="cfg-ac-dist" type="number" step="0.1" onchange="cfgChanged()">
    <label>清晰度阈值</label><input id="cfg-sharp" type="number" step="0.1" onchange="cfgChanged()">
    <label>充分样本数</label><input id="cfg-good" type="number" onchange="cfgChanged()">
  </details>

  <button class="cfg-save" id="cfg-save-btn" onclick="saveConfig()" disabled>保存配置</button>

  <h3>结果</h3>
  <div id="result-area">
    <div class="result-block"><span class="label">暂无结果</span></div>
  </div>

  <h3>日志</h3>
  <div id="log-panel"><div class="line">就绪</div></div>
</div>
</div>

<!-- review modal -->
<div class="modal-overlay" id="review-modal">
  <div class="modal">
    <h2>&#9888; 内参标定确认</h2>
    <p style="font-size:13px;color:#a6adc8">请检查重投影误差画面。确认写入后，config/calibration.yaml 将更新为新内参。</p>
    <div class="compare">
      <div><h4>新结果 (camera_calibration.yaml)</h4>
        <pre id="new-intrinsics">--</pre></div>
      <div><h4>当前配置 (config/calibration.yaml)</h4>
        <pre id="old-intrinsics">--</pre></div>
    </div>
    <p style="font-size:11px;color:#f38ba8;margin-top:8px">注意：未确认时，后续手眼流程仍使用旧内参</p>
    <div class="actions">
      <button class="btn-reject" onclick="reviewAction('reject_intrinsics')">放弃写回</button>
      <button class="btn-accept" onclick="reviewAction('accept_intrinsics')">确认写回</button>
    </div>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
// ---- state ----
let currentTask = null;
let session = {state:'idle',task_type:'',phase:'空闲',auto_collect:false};
let configDirty = false;
let knownStream = false;

const taskDefs = [
  {id:'intrinsic_calibration', name:'内参标定', desc:'采集棋盘图像,计算相机内参矩阵与畸变系数'},
  {id:'intrinsic_validation', name:'内参验证', desc:'实时显示重投影误差,验证当前内参精度'},
  {id:'intrinsic_recalibration', name:'重新内参标定', desc:'备份旧结果后重新执行内参标定'},
  {id:'handeye_calibration', name:'手眼标定', desc:'采集手眼数据,计算相机与云台间变换关系'},
  {id:'handeye_validation', name:'手眼验证', desc:'位置一致性验证,评估手眼标定结果质量'},
  {id:'handeye_recalibration', name:'重新手眼标定', desc:'备份旧结果后重新执行手眼标定'},
];

// ---- toast ----
let toastTimer = null;
function toast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg; t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 1200);
}

// ---- escape html ----
function esc(s) { return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/"/g,'&quot;'); }

// ---- init ----
function init() {
  renderTaskList();
  fetchConfig();
  fetchSession();
  setInterval(fetchSession, 500);
  document.body.focus();
}

function renderTaskList() {
  const el = document.getElementById('task-list');
  el.innerHTML = taskDefs.map(t =>
    '<div class="task-card" id="task-'+t.id+'" onclick="selectTask(\''+t.id+'\')">'+
    '<div class="name">'+esc(t.name)+'</div>'+
    '<div class="desc">'+esc(t.desc)+
      (t.id.startsWith('handeye') ? ' <span title="需要串口">&#128268;</span>' : '')+
    '</div>'+
    '<span class="status status-ready">可启动</span></div>'
  ).join('');
}

function selectTask(id) {
  currentTask = id;
  document.querySelectorAll('.task-card').forEach(c => c.classList.remove('active'));
  const card = document.getElementById('task-'+id);
  if (card) card.classList.add('active');

  const canStart = (session.state === 'idle' || session.state === 'succeeded' ||
                    session.state === 'failed' || session.state === 'stopped');
  document.getElementById('btn-start').disabled = !canStart;
  document.getElementById('btn-stop').disabled = (session.state === 'idle');
  document.getElementById('btn-recal').disabled = false;

  const isRecal = id.includes('recalibration');
  const isHandeye = id.startsWith('handeye');
  let hint = isRecal ? '重新标定将先备份旧结果' : '选择任务后点击开始';
  if (isHandeye) {
    if (session.serial_status === 'failed')
      hint = '⚠ 串口不可用，启动将失败';
    else
      hint = '启动任务时将自动打开串口';
  }
  document.getElementById('hint-text').textContent = hint;
}

// ---- API calls ----
async function fetchSession() {
  try {
    const r = await fetch('/api/session');
    session = await r.json();
    updateUI();
  } catch(e) { document.getElementById('dot').className = 'dot off'; }
}

async function fetchConfig() {
  try {
    const r = await fetch('/api/config');
    const cfg = await r.json();
    populateConfig(cfg);
  } catch(e) {}
}

function populateConfig(cfg) {
  setVal('cfg-device', cfg.device);
  setVal('cfg-pattern', cfg.pattern);
  document.getElementById('cfg-rows').value = cfg.pattern_rows || 8;
  document.getElementById('cfg-cols').value = cfg.pattern_cols || 8;
  document.getElementById('cfg-square').value = cfg.square_size || 35;
  document.getElementById('cfg-cam-save').value = cfg.camera_calib_save_path || '';
  document.getElementById('cfg-he-save').value = cfg.handeye_calib_save_path || '';
  if (cfg.Serial) {
    document.getElementById('cfg-serial-port').value = cfg.Serial.port_name || '';
    document.getElementById('cfg-serial-baud').value = cfg.Serial.baud_rate || 115200;
  }
  if (cfg.HIK) {
    document.getElementById('cfg-hik-exp').value = cfg.HIK.exposure_time || 2000;
    document.getElementById('cfg-hik-gain').value = cfg.HIK.gain || 16.9;
  }
  if (cfg.IMG) {
    document.getElementById('cfg-img-path').value = cfg.IMG.images_path || '';
  }
  document.getElementById('cfg-ac-int').value = cfg.auto_collect_interval_ms || 50;
  document.getElementById('cfg-ac-dist').value = cfg.auto_collect_param_distance || 0.2;
  document.getElementById('cfg-sharp').value = cfg.auto_collect_sharpness_threshold || 40;
  document.getElementById('cfg-good').value = cfg.auto_collect_goodenough_samples || 40;
  configDirty = false;
  document.getElementById('cfg-save-btn').disabled = true;
}

function setVal(id, val) {
  const el = document.getElementById(id);
  if (el) el.value = (val != null) ? val : '';
}

function cfgChanged() { configDirty = true; document.getElementById('cfg-save-btn').disabled = false; }

async function saveConfig() {
  const cfg = {
    device: document.getElementById('cfg-device').value,
    pattern: document.getElementById('cfg-pattern').value,
    pattern_rows: parseInt(document.getElementById('cfg-rows').value)||8,
    pattern_cols: parseInt(document.getElementById('cfg-cols').value)||8,
    square_size: parseFloat(document.getElementById('cfg-square').value)||35,
    camera_calib_save_path: document.getElementById('cfg-cam-save').value,
    handeye_calib_save_path: document.getElementById('cfg-he-save').value,
    Serial: {
      port_name: document.getElementById('cfg-serial-port').value,
      baud_rate: parseInt(document.getElementById('cfg-serial-baud').value)||115200
    },
    HIK: {
      exposure_time: parseInt(document.getElementById('cfg-hik-exp').value)||2000,
      gain: parseFloat(document.getElementById('cfg-hik-gain').value)||16.9
    },
    IMG: {
      images_path: document.getElementById('cfg-img-path').value
    },
    auto_collect_interval_ms: parseInt(document.getElementById('cfg-ac-int').value)||50,
    auto_collect_param_distance: parseFloat(document.getElementById('cfg-ac-dist').value)||0.2,
    auto_collect_sharpness_threshold: parseFloat(document.getElementById('cfg-sharp').value)||40,
    auto_collect_goodenough_samples: parseInt(document.getElementById('cfg-good').value)||40
  };
  try {
    const r = await fetch('/api/config', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(cfg)
    });
    if (r.ok) { configDirty = false; document.getElementById('cfg-save-btn').disabled = true; toast('配置已保存'); }
    else toast('保存失败');
  } catch(e) { toast('保存失败: '+e.message); }
}

async function startTask() {
  if (!currentTask) { toast('请先选择任务'); return; }
  try {
    const r = await fetch('/api/tasks/start', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({task_type: currentTask})
    });
    const txt = await r.text();
    if (r.ok) { toast('启动指令已提交'); fetchSession(); }
    else { toast('启动失败: '+txt); fetchSession(); }
  } catch(e) { toast('请求失败'); }
}

async function stopTask() {
  try {
    await fetch('/api/tasks/stop', {method: 'POST'});
    toast('已发送停止指令');
    fetchSession();
  } catch(e) {}
}

async function recalTask() {
  if (!currentTask) return;
  let recalId = currentTask;
  if (currentTask === 'intrinsic_calibration') recalId = 'intrinsic_recalibration';
  else if (currentTask === 'handeye_calibration') recalId = 'handeye_recalibration';
  currentTask = recalId;
  await startTask();
}

async function sendAction(action) {
  try {
    const r = await fetch('/api/tasks/action', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({action: action})
    });
    const txt = await r.text();
    toast(txt || action);
    fetchSession();
  } catch(e) { toast('请求失败'); }
}

async function reviewAction(action) {
  try {
    const r = await fetch('/api/tasks/action', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({action: action})
    });
    const txt = await r.text();
    toast(txt || 'OK');
    document.getElementById('review-modal').classList.remove('show');
    fetchSession();
  } catch(e) { toast('请求失败'); }
}

// ---- UI update ----
function updateUI() {
  const dot = document.getElementById('dot');
  dot.className = 'dot' + (session.state === 'idle' ? '' : '');

  document.getElementById('hdr-device').textContent = session.device_type || '--';
  document.getElementById('hdr-phase').textContent = session.phase || '空闲';
  document.getElementById('hdr-uptime').textContent = (session.uptime_s || 0) + 's';

  // serial status
  const serEl = document.getElementById('hdr-serial');
  const st = session.serial_status || 'closed';
  if (st === 'open') {
    serEl.textContent = '已连接';
    serEl.style.color = '#a6e3a1';
  } else if (st === 'failed') {
    serEl.textContent = '打开失败';
    serEl.style.color = '#f38ba8';
    serEl.title = session.serial_error_message || '';
  } else {
    serEl.textContent = '未启用';
    serEl.style.color = '#6c7086';
    serEl.title = '';
  }

  // stream
  if (!knownStream && session.state !== 'idle') {
    document.getElementById('video-placeholder').style.display = 'none';
    document.getElementById('stream-img').style.display = 'block';
    document.getElementById('stream-img').src = '/stream?window=workbench&t='+Date.now();
    document.getElementById('stream-hud').style.display = 'block';
    knownStream = true;
  }
  if (session.state === 'idle') {
    document.getElementById('stream-hud').textContent = '等待任务启动';
  } else {
    document.getElementById('stream-hud').textContent =
      '阶段: '+session.phase+' | 样本: '+session.sample_count+
      (session.task_type === 'intrinsic_calibration' ? '/'+session.goodenough_samples : '')+
      (session.task_type === 'handeye_calibration' ? ' | 手动采集' : '')+
      (session.auto_collect ? ' | 自动采集中' : '');
  }

  // action buttons
  const allowed = session.allowed_actions || [];
  const aa = (id, v) => { const b = document.getElementById(id); if (b) b.disabled = !v; };
  aa('act-collect', allowed.includes('collect'));
  aa('act-auto', allowed.includes('toggle_auto_collect'));
  aa('act-skip', allowed.includes('skip'));
  aa('act-compute', allowed.includes('compute'));
  aa('act-reset', allowed.includes('reset'));

  // auto collect button text
  const ab = document.getElementById('act-auto');
  if (ab) {
    ab.hidden = session.task_type !== 'intrinsic_calibration';
    ab.textContent = session.auto_collect ? '自动采集:开' : '自动采集:关';
  }

  // task cards
  document.getElementById('btn-start').disabled = !(session.state === 'idle' ||
    session.state === 'succeeded' || session.state === 'failed' || session.state === 'stopped');
  document.getElementById('btn-stop').disabled = (session.state === 'idle' ||
    session.state === 'succeeded' || session.state === 'failed' || session.state === 'stopped');

  // update task card statuses
  taskDefs.forEach(t => {
    const card = document.getElementById('task-'+t.id);
    if (!card) return;
    const st = card.querySelector('.status');
    if (session.task_type === t.id && session.state !== 'idle') {
      st.textContent = session.state === 'running' ? '运行中' :
                       session.state === 'review_pending' ? '待确认' :
                       session.state === 'succeeded' ? '已完成' :
                       session.state === 'failed' ? '失败' :
                       session.state === 'stopped' ? '已停止' : session.state;
      st.className = 'status ' + (
        session.state === 'running' ? 'status-running' :
        session.state === 'review_pending' ? 'status-running' :
        session.state === 'succeeded' ? 'status-done' :
        'status-err');
    } else {
      st.textContent = '可启动'; st.className = 'status status-ready';
    }
  });

  // review modal
  if (session.state === 'review_pending') {
    document.getElementById('review-modal').classList.add('show');
    document.getElementById('new-intrinsics').textContent =
      'camera_matrix: '+JSON.stringify(session.new_camera_matrix)+'\n'+
      'distort_coeffs: '+JSON.stringify(session.new_distort_coeffs);
    document.getElementById('old-intrinsics').textContent =
      'camera_matrix: '+JSON.stringify(session.old_camera_matrix)+'\n'+
      'distort_coeffs: '+JSON.stringify(session.old_distort_coeffs);
  } else {
    document.getElementById('review-modal').classList.remove('show');
  }

  // results area
  if (session.result_path) {
    document.getElementById('result-area').innerHTML =
      '<div class="result-block"><span class="label">结果文件</span> '+
      '<span class="value">'+esc(session.result_path)+'</span></div>';
  }

  // logs
  if (session.recent_logs && session.recent_logs.length > 0) {
    const lp = document.getElementById('log-panel');
    lp.innerHTML = session.recent_logs.map(l =>
      '<div class="line">'+esc(l)+'</div>').join('');
    lp.scrollTop = lp.scrollHeight;
  }
}

// keyboard shortcuts
document.body.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  let code;
  if (e.key === 'Escape') code = 27;
  else if (e.key === 'Enter') code = 13;
  else if (e.key === 'Backspace') code = 8;
  else if (e.key.length === 1) code = e.key.charCodeAt(0);
  else return;
  // 工作台动作走同一 API，避免 /key 和 /api/tasks/action 重复执行。
  if (code === 27) { stopTask(); return; }
  if (e.key === 's') { sendAction('collect'); return; }
  if (e.key === 'a') { sendAction('toggle_auto_collect'); return; }
  if (e.key === 'c') { sendAction('compute'); return; }
  if (e.key === 'r') { sendAction('reset'); return; }
  fetch('/key', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({key: code})
  }).catch(() => {});
});

init();
</script>
</body>
</html>)html";

} // namespace qd
