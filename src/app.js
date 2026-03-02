// ==========================================
// Edge-Server Polling Logic
// ==========================================

const API_STATUS_URL = '/status';
const POLLING_INTERVAL = 500; // Poll every 500ms

async function fetchSystemStatus() {
    try {
        const response = await fetch(API_STATUS_URL);
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();
        updateDashboard(data);
    } catch (error) {
        console.error("📡 Connection to Edge Server failed:", error);
    }
}

function updateDashboard(data) {
    // 1. Update Parking Slots (JSON returns array: [False, True, ...])
    // True = Occupied, False = Vacant
    for (let i = 0; i < 4; i++) {
        const slotElement = document.getElementById(`slot_${i}`);

        // FIX: null-check before accessing child elements
        if (!slotElement) continue;

        const badgeElement = slotElement.querySelector('.status-badge');
        if (!badgeElement) continue;

        const isOccupied = data.parking[i];

        if (isOccupied) {
            slotElement.className = 'slot occupied';
            badgeElement.innerText = 'Occupied';
        } else {
            slotElement.className = 'slot vacant';
            badgeElement.innerText = 'Vacant';
        }
    }

    // 2. Update Gates/Servos (JSON returns array: ["Closed", "Open"])
    const entranceGate = document.getElementById('gate_entrance');
    const exitGate     = document.getElementById('gate_exit');

    // Update Entrance
    const entranceState = data.servo[0] || "Closed";
    entranceGate.innerText  = entranceState;
    entranceGate.className  = `gate-indicator ${entranceState.toLowerCase()}`;

    // FIX: was writing to entranceGate instead of exitGate
    const exitState = data.servo[1] || "Closed";
    exitGate.innerText = exitState;
    exitGate.className = `gate-indicator ${exitState.toLowerCase()}`;
}

// Start the polling loop when the page loads
window.addEventListener('DOMContentLoaded', () => {
    console.log("🚀 Dashboard Initialized. Starting telemetry polling...");
    setInterval(fetchSystemStatus, POLLING_INTERVAL);
});
