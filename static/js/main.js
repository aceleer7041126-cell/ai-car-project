// main.js — 搖桿 / 伺服 / 狀態輪詢

// ---- 方向控制：按住持續送指令，放開停止 ----
let holdTimer = null;

function sendMove(action) {
    const spd = document.getElementById('speed');
    fetch('/api/move', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action, speed: spd ? Number(spd.value) : 60 })
    });
}

document.querySelectorAll('.dpad').forEach(btn => {
    const act = btn.dataset.act;
    // 滑鼠 / 觸控按住
    const start = e => {
        e.preventDefault();
        sendMove(act);
        if (act !== 'stop') {
            holdTimer = setInterval(() => sendMove(act), 200);
        }
    };
    const end = () => {
        if (holdTimer) { clearInterval(holdTimer); holdTimer = null; }
        if (act !== 'stop') sendMove('stop');
    };
    btn.addEventListener('mousedown', start);
    btn.addEventListener('touchstart', start, { passive: false });
    btn.addEventListener('mouseup', end);
    btn.addEventListener('mouseleave', end);
    btn.addEventListener('touchend', end);
});

// ---- 伺服馬達 ----
document.querySelectorAll('[data-servo]').forEach(btn => {
    btn.addEventListener('click', () => {
        fetch('/api/servo', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ direction: btn.dataset.servo })
        });
    });
});

// ---- 速度滑桿 ----
const speedRange = document.getElementById('speed');
const spdVal = document.getElementById('spd-val');
speedRange.addEventListener('input', () => { spdVal.textContent = speedRange.value; });
speedRange.addEventListener('change', () => {
    fetch('/api/speed', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed: Number(speedRange.value) })
    });
});

// ---- 自動跟線開關 ----
let autoOn = false;
const autoBtn = document.getElementById('btn-auto');
autoBtn.addEventListener('click', async () => {
    autoOn = !autoOn;
    await fetch('/api/auto', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ on: autoOn })
    });
    autoBtn.textContent = autoOn ? '⏹ 停止自動跟線' : '▶ 啟動自動跟線';
    autoBtn.classList.toggle('on', autoOn);
});

// ---- 狀態輪詢（每 0.5 秒） ----
async function poll() {
    try {
        const r = await fetch('/api/status');
        const d = await r.json();
        document.getElementById('st-action').textContent = d.action;
        document.getElementById('st-cte').textContent = d.cte === null ? '--' : d.cte;
        let dir = '--';
        if (d.cte !== null) dir = d.cte > 0 ? '偏右→左修' : (d.cte < 0 ? '偏左→右修' : '置中');
        document.getElementById('st-dir').textContent = dir;
        const ad = d.auto_detail || {};
        document.getElementById('st-u').textContent =
            (ad.u !== undefined) ? `u=${ad.u} L=${ad.left} R=${ad.right}` : '--';
        document.getElementById('st-sh').textContent = d.servo_h;
        document.getElementById('st-sv').textContent = d.servo_v;
        document.getElementById('st-yolo').textContent = d.yolo ? '已載入' : '未載入';
        const objs = d.objects || {};
        document.getElementById('st-obj').textContent =
            Object.keys(objs).length ? Object.entries(objs).map(([k, v]) => `${k}:${v}`).join(' ') : '無';
        document.getElementById('st-mock').textContent = d.mock ? 'MOCK（無硬體）' : '硬體';
    } catch (e) { /* 忽略 */ }
}
setInterval(poll, 500);
poll();
