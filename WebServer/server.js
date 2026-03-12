const express = require("express");
const http = require("http");
const { Server } = require("socket.io");
const cors = require("cors");
const ioClient = require("socket.io-client");

const app = express();
app.use(cors());
app.use(express.static("public"));

const server = http.createServer(app);
const io = new Server(server);

const PORT = 3000;
const PI_ID = process.env.PI_ID || "PI_UNKNOWN";
const PEERS = (process.env.PEERS || "").split(",").filter(Boolean);

let identifiedDrones = [];
let peerSockets = [];

function connectToPeers() {
    PEERS.forEach(peer => {
        const socket = ioClient(`http://${peer}:3000`);

        socket.on("connect", () => {
            console.log(`${PI_ID} connected to peer ${peer}`);
        });

        socket.on("syncDrones", (drones) => {
            identifiedDrones = drones;
            io.emit("updateDrones", identifiedDrones);
        });

        peerSockets.push(socket);
    });
}

io.on("connection", (socket) => {
    socket.emit("initData", {
        piId: PI_ID,
        drones: identifiedDrones
    });

    socket.on("newDrone", (drone) => {
        identifiedDrones.push(drone);

        io.emit("updateDrones", identifiedDrones);

        peerSockets.forEach(ps =>
            ps.emit("syncDrones", identifiedDrones)
        );
    });
});

server.listen(PORT, () => {
    console.log(`${PI_ID} running on port ${PORT}`);
    setTimeout(connectToPeers, 2000);
});
