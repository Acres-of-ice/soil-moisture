// Initialize function
$(document).ready(function(){
    startDataInterval();
});   

function startDataInterval() {
    // Fetch sensor data every 5 seconds
    setInterval(getSensorData, 5000);
}

function getSensorData() {
    $.getJSON('/sensorReadings', function(data) {
        console.log("Received sensor readings:", data);
        updateSensorReadings(data);
    });
}

function updateSensorReadings(data) {

    updateSensorValue('counter', data.cnt, '');
    updateSensorValue('moisture_a', data.moisture_a, '%');
    updateSensorValue('moisture_b', data.moisture_b, '%');   
}

function updateSensorValue(id, value, unit) {
    const element = document.getElementById(id);
    if (element) {
        if (value !== undefined && value !== null) {
            const formatted = Number.isInteger(value) ? value : Number(value).toFixed(1);
            element.textContent = formatted + (unit ? ' ' + unit : '');
        } else {
            element.textContent = '--';
        }
    } else {
        console.error('Element not found for sensor:', id);
    }
}

