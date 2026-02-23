async function fetchStatus() {
    const res = await fetch("/status");
    const data = await res.json();

    const parkingDiv = document.getElementById("parking");
    parkingDiv.innerHTML = "";

    data.parking.forEach((status, index) => {
        const slot = document.createElement("div");
        slot.className = "slot " + (status ? "occupied" : "free");
        slot.innerText = "Slot " + (index+1) +
                         (status ? " - Occupied" : " - Free");
        parkingDiv.appendChild(slot);
    });

    const servoDiv = document.getElementById("servo-status");
    servoDiv.innerHTML = "";

    data.servo.forEach((s, index) => {
        const servo = document.createElement("div");
        servo.className = "servo";
        servo.innerText = "Servo " + (index+1) + ": " + s;
        servoDiv.appendChild(servo);
    });

    document.getElementById("ssd").src = data.ssd1306;
}

setInterval(fetchStatus, 1000);
fetchStatus();