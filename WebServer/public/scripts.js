const socket = io();
let drones = [];
let markers = [];
let degree = 0;
const map = L.map('map').setView([0, 0], 2);

window.addEventListener("load",init)
socket.on("initData", (data) => {
    drones = data.drones;
    render();
});

socket.on("updateDrones", (data) => {
    drones = data;
    render();
});

socket.on("newDegree", (degree) => {
    updateDiagram(degree);
})

socket.on("frame", (b64) => {
    const videoElement = document.getElementById('droneVideo');
    videoElement.src = `data:image/jpeg;base64,${b64}`;
})
function init() {
    showMyLocation();
    startCameraFeed();
}


function getCurrentPosition(options = {}) {
    return new Promise((resolve, reject) => {
        if (!navigator.geolocation) {
            reject(new Error("Geolocation is not supported by this browser."));
            return;
        }

        navigator.geolocation.getCurrentPosition(
            (position) => {
                resolve({
                    lat: position.coords.latitude,
                    lng: position.coords.longitude,
                    accuracy: position.coords.accuracy
                });
            },
            (error) => {
                reject(error);
            },
            options
        );
    });
}

async function showMyLocation() {
    try {
        const position = await getCurrentPosition();

        if (!position || !position.lat || !position.lng) {
            throw new Error("Invalid location data");
        }
        var meIcon = L.icon({
            iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-violet.png',
            shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
        });


        map.setView([position.lat, position.lng], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; OpenStreetMap contributors'
        }).addTo(map);
        L.marker([position.lat, position.lng], {icon: meIcon})
            .addTo(map)
            .bindPopup("You are here!")
            .openPopup();

    } catch (error) {
        console.error("Location error:", error.message);
    }
}


async function addDrone() { // Simulate Drone Detection

    const position = await getCurrentPosition()
    const latOffset = (Math.random() - 0.8) * 0.02; // Random offset until dronedetction is implemented
    const lngOffset = (Math.random() - 0.8) * 0.02;
    const droneLat = position.lat + latOffset;
    const droneLng = position.lng + lngOffset;
    if (droneLat == null){
        throw new Error("Empty latitude")
    }
    if (droneLng == null){
        throw new Error("Empty longitude")
    }
    const newDrone = {
        id: "Drone_" + Math.floor(Math.random() * 1000),
        type: "Type A",
        time: new Date().toLocaleTimeString(),
        lat: droneLat,
        lng: droneLng
    };
    L.marker([newDrone.lat, newDrone.lng])
        .addTo(map)
        .bindPopup("Drone:"+newDrone.id)
        .openPopup();

    socket.emit("newDrone", newDrone);
}

async function startCameraFeed() { //To be implemented with proper camera feed
    const videoElement = document.getElementById('droneVideo');

    try {
        videoElement.srcObject = await navigator.mediaDevices.getUserMedia({ video: true, audio: false });
        videoElement.play();
    } catch (err) {
        console.error("Error accessing media devices:", err);
    }
}


function render() {
    const list = document.getElementById("droneList");
    list.innerHTML = "";

    markers.forEach(m => map.removeLayer(m));
    markers = [];

    drones.forEach(d => {
        const div = document.createElement("div");
        div.className = "drone-item";
        div.innerHTML = `<span>${d.id}</span><span>${d.time}</span>`;
        list.appendChild(div);

        if (d.lat && d.lng) {
            const marker = L.marker([d.lat, d.lng])
                .addTo(map)
                .bindPopup(`<b>${d.id}</b><br>${d.time}`);

            markers.push(marker);
        }
    });
}

function updateDiagram(degrees) {
    const center_x = 150;
    const center_y = 150;
    const radius = 100;
    console.log("updateDiagram kjører")
    degrees = parseFloat(degrees);
    let radians = degrees * (Math.PI / 180);
    const x = center_x + radius * Math.cos(radians);
    const y = center_y + radius * Math.sin(radians);

    const dot = document.getElementById("movingDot");
    dot.setAttribute("cx", x);
    dot.setAttribute("cy", y);

    const line = document.getElementById("radiusLine");
    line.setAttribute("x2", x);
    line.setAttribute("y2", y);

    const degreeText =document.getElementById("DegreeText")
    degreeText.value = `Degree: ${degrees}`
}



if (typeof module !== 'undefined' && module.exports) {
    module.exports = { getCurrentPosition, addDrone, updateDiagram, render, init, startCameraFeed };
}