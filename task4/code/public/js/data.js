var socket = io();
window.addEventListener("load", function () {
    var lightbox = document.getElementById("light");
    lightbox.addEventListener("change", function () {
        socket.emit("light", Number(this.checked));
    });
});
socket.on('light', function (data) {
    document.getElementById("light").checked = !!data;
});
socket.on('data', function (data) {
    document.getElementById("distance").innerHTML = "Distance: " + (data.distance || "N/A");
    document.getElementById("leftTicks").innerHTML = "Left Ticks: " + (data.left_ticks || "N/A");
    document.getElementById("rightTicks").innerHTML = "Right Ticks: " + (data.right_ticks || "N/A");
});
socket.on('connect', () => {
    console.log('Connected to server');
});
socket.on('disconnect', () => {
    console.log('Disconnected from server');
});