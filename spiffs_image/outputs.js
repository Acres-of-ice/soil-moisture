// Global variables
var AUTO_mode = false;
var SPRAY_mode = false;
var DRAIN_mode = false;

// Initialize functions
$(document).ready(function(){
    getInitialState();
    startDataIntervals();
});   

function getInitialState() {
    $.getJSON('/getInitialState', function(data) {
        console.log("Initial state:", data);
        updateStates(data);
    });
}

function startDataIntervals() {
    setInterval(getControlData, 3000);
}

function getControlData() {
    $.getJSON('/outputVal', function(data) {
        console.log("Received control data:", data);
        updateStates(data);
    });
}

function updateStates(data) {
    document.getElementById('current-state').textContent = data.STATE;

    // Update global variables
    AUTO_mode = Boolean(data.AUTO);
    SPRAY_mode = Boolean(data.SPRAY);
    DRAIN_mode = Boolean(data.DRAIN);

    // Update UI
    updateUI('AUTO', AUTO_mode);
    updateUI('SPRAY', SPRAY_mode);
    updateUI('DRAIN', DRAIN_mode);

    console.log("Updated states - AUTO:", AUTO_mode, "SPRAY:", SPRAY_mode, "DRAIN:", DRAIN_mode);
}

function updateUI(mode, state) {
    var checkbox = document.getElementById(mode + '_checkbox');
    var statusElement = document.getElementById(mode + '_status');
    
    if (checkbox && statusElement) {
        checkbox.checked = state;
        statusElement.textContent = state ? 'ON' : 'OFF';
    } else {
        console.error('UI elements not found for', mode);
    }
}

function setMode(mode) {
    var checkbox = document.getElementById(mode + '_checkbox');
    if (checkbox) {
        var newState = checkbox.checked;
        
        // If AUTO mode is being enabled, disable other modes
        if (mode === 'AUTO' && newState) {
            SPRAY_mode = false;
            DRAIN_mode = false;
            updateUI('SPRAY', false);
            updateUI('DRAIN', false);
        }
        
        // If AUTO mode is on, prevent changing other modes
        if (AUTO_mode && mode !== 'AUTO') {
            alert('Cannot change modes when AUTO is enabled');
            checkbox.checked = !newState;  // Revert the checkbox
            return;
        }

        var data = JSON.stringify({
            mode: mode,
            state: newState ? 1 : 0
        });

        $.ajax({
            url: '/setMode',
            type: 'POST',
            contentType: 'application/json',
            data: data,
            success: function(response) {
                console.log("Mode set successfully:", mode, newState);
                // Update the global variable
                window[mode + '_mode'] = newState;
            },
            error: function(xhr, status, error) {
                console.error("Failed to set mode:", mode, error);
                // Revert the checkbox if the server update failed
                checkbox.checked = !newState;
                updateUI(mode, !newState);
            }
        });
    } else {
        console.error('Checkbox not found for', mode);
    }
}
