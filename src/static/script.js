const POLL_MS = 600;

// ---- Clock (local browser time) ----
function tickClock() {
    const now  = new Date();
    const h    = String(now.getHours()).padStart(2,'0');
    const m    = String(now.getMinutes()).padStart(2,'0');
    const s    = String(now.getSeconds()).padStart(2,'0');
    const days = ['Sunday','Monday','Tuesday','Wednesday','Thursday','Friday','Saturday'];
    const months = ['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'];
    document.getElementById('clock').textContent = `${h}:${m}:${s}`;
    document.getElementById('date').textContent  =
        `${days[now.getDay()]}, ${now.getDate()} ${months[now.getMonth()]} ${now.getFullYear()}`;
}
setInterval(tickClock, 1000);
tickClock();

// ---- Status Poll ----
async function pollStatus() {
    try {
        const res  = await fetch('/api/status');
        const data = await res.json();
        updateDashboard(data);
    } catch (e) {
        console.warn('Poll failed:', e);
    }
}

function updateDashboard(data) {
    // Connection pills
    setPill('pill-cam', data.cam_online);
    setPill('pill-sub', data.sub_online);

    // Parking slots
    for (let i = 0; i < 4; i++) {
        const card   = document.getElementById(`slot-${i}`);
        const status = card.querySelector('.slot-status');
        const freq   = document.getElementById(`freq-${i}`);
        if (!card) continue;
        const occ = data.parking[i];
        card.classList.toggle('occupied', occ);
        status.textContent = occ ? 'Occupied' : 'Vacant';
        if (freq) freq.textContent = `${data.freq[i] ?? 0} Hz`;
    }

    // Gates
    setGate('gate-in',  data.servo[0]);
    setGate('gate-out', data.servo[1]);

    // Last seen
    const ls = document.getElementById('last-seen');
    if (ls) ls.textContent = data.last_seen || '—';
}

function setPill(id, online) {
    const el = document.getElementById(id);
    if (!el) return;
    el.classList.toggle('online',  online);
    el.classList.toggle('offline', !online);
}

function setGate(id, state) {
    const el = document.getElementById(id);
    if (!el) return;
    const isOpen = state === 'Open';
    el.textContent = state;
    el.classList.toggle('open', isOpen);
}

// ---- Manual Gate Trigger ----
async function triggerGate(type) {
    try {
        await fetch(`/api/gate/${type}`);
    } catch (e) {
        console.warn('Gate trigger failed:', e);
    }
}

// Start polling
setInterval(pollStatus, POLL_MS);
pollStatus();