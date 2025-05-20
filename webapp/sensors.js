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
    updateSensorValue('temp', data.temp, '°C');
    updateSensorValue('cnt', data.cnt, '');
    updateSensorValue('moisture_a', data.moisture_a, '%');
    updateSensorValue('moisture_b', data.moisture_b, '%');  
    updateSensorValue('wind', data.wind, 'm/s');
    updateSensorValue('pressure', data.pressure, 'm');
    updateSensorValue('water_temp', data.water_temp, '°C');
    updateSensorValue('discharge', data.discharge, 'l/min');
}

function updateSensorValue(id, value, unit) {
    var element = document.getElementById(id);
    if (element) {
        if (value !== undefined && value !== null) {
            // Format the number to 1 decimal place if it's not an integer
            var formattedValue = Number.isInteger(value) ? value : Number(value).toFixed(1);
            element.textContent = formattedValue + (unit ? ' ' + unit : '');
        } else {
            element.textContent = '--';
        }
    } else {
        console.error('Element not found for sensor:', id);
    }
}
