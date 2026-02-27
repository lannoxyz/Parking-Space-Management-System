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
    // 1. Update Parking Slots (Assuming JSON returns array: [False, True, ...])
    // True = Occupied, False = Vacant
    for (let i = 0; i < 4; i++) {
        const slotElement = document.getElementById(`slot_${i}`);
        const badgeElement = slotElement.querySelector('.status-badge');
        
        const isOccupied = data.parking[i]; 
        
        if (isOccupied) {
            slotElement.className = 'slot occupied';
            badgeElement.innerText = 'Occupied';
        } else {
            slotElement.className = 'slot vacant';
            badgeElement.innerText = 'Vacant';
        }
    }

    // 2. Update Gates/Servos (Assuming JSON returns array: ["Closed", "Open"])
    const entranceGate = document.getElementById('gate_entrance');
    const exitGate = document.getElementById('gate_exit');

    // Update Entrance
    entranceGate.innerText = data.servo[0];
    entranceGate.className = `gate-indicator ${data.servo[0].toLowerCase()}`;

    // Update Exit
    entranceGate.innerText = data.servo[1] || "Closed";
    exitGate.className = `gate-indicator ${(data.servo[1] || "Closed").toLowerCase()}`;
}

// Start the polling loop when the page loads
window.addEventListener('DOMContentLoaded', () => {
    console.log("🚀 Dashboard Initialized. Starting telemetry polling...");
    setInterval(fetchSystemStatus, POLLING_INTERVAL);
});
