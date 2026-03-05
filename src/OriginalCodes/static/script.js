const POLL_MS = 600;

// ---- Clock ----
function tickClock() {
    const now    = new Date();
    const h      = String(now.getHours()).padStart(2, '0');
    const m      = String(now.getMinutes()).padStart(2, '0');
    const s      = String(now.getSeconds()).padStart(2, '0');
    const days   = ['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
    const months = ['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'];
    const el_t   = document.getElementById('clock');
    const el_d   = document.getElementById('date');
    if (el_t) el_t.textContent = `${h}:${m}:${s}`;
    if (el_d) el_d.textContent = `${days[now.getDay()]} ${now.getDate()} ${months[now.getMonth()]} ${now.getFullYear()}`;
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
    // Connection indicators
    setOnline('dot-cam', 'val-cam', data.cam_online);
    setOnline('dot-sub', 'val-sub', data.sub_online);

    // Parking slots
    let vacantCount = 0, occCount = 0;
    for (let i = 0; i < 4; i++) {
        const card  = document.getElementById(`slot-${i}`);
        const state = document.getElementById(`slot-state-${i}`);
        const freq  = document.getElementById(`freq-${i}`);
        if (!card) continue;

        const occ = data.parking?.[i] ?? false;
        card.classList.toggle('occupied', occ);
        if (state) state.textContent = occ ? 'OCCUPIED' : 'VACANT';
        if (freq)  freq.textContent  = `${data.freq?.[i] ?? 0} Hz`;

        occ ? occCount++ : vacantCount++;
    }

    const cv = document.getElementById('count-vacant');
    const co = document.getElementById('count-occ');
    if (cv) cv.textContent = vacantCount;
    if (co) co.textContent = occCount;

    // Gate — servo is now a scalar string ("Open" / "Closed")
    const gateState = typeof data.servo === 'string'
        ? data.servo
        : (Array.isArray(data.servo) ? data.servo[0] : 'Closed');
    setGate('gate-main', gateState);

    // Last seen
    const ls = document.getElementById('last-seen');
    if (ls) ls.textContent = data.last_seen || '—';
}

function setOnline(dotId, valId, online) {
    const dot = document.getElementById(dotId);
    const val = document.getElementById(valId);
    if (dot) {
        dot.classList.toggle('online',  !!online);
        dot.classList.toggle('offline', !online);
    }
    if (val) {
        val.textContent = online ? 'ONLINE' : 'OFFLINE';
        val.classList.toggle('online', !!online);
    }
}

function setGate(id, state) {
    const el = document.getElementById(id);
    if (!el) return;
    const isOpen = state === 'Open';
    el.textContent = isOpen ? 'OPEN' : 'CLOSED';
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

setInterval(pollStatus, POLL_MS);
pollStatus();