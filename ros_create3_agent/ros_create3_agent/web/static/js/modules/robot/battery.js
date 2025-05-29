// robot/battery.js - Battery display functions


/**
 * Update battery status display
 * @param {Object} battery - Battery status data
 */
export function updateBatteryStatus(battery) {
  const elements = window.appElements;
  const batteryData = battery || {};

  // Update battery level
  elements.batteryLevel.textContent = batteryData.level || "Unknown";

  // Update battery status text and class
  const batteryStatus = document.getElementById('battery-status');
  const status_text = batteryData.status || "Unknown";
  batteryStatus.textContent = status_text;
  batteryStatus.className = 'battery-status';
  batteryStatus.classList.add('battery-status-' + status_text);

  // Update battery voltage and current (2 decimal places)
  const batteryVoltage = document.getElementById('battery-voltage');
  const batteryCurrent = document.getElementById('battery-current');
  const voltage = batteryData.voltage !== null ? batteryData.voltage.toFixed(2) : "0.00";
  const current = batteryData.current !== null ? batteryData.current.toFixed(2) : "0.00";
  batteryVoltage.textContent = `${voltage} V`;
  batteryCurrent.textContent = `${current} A`;
}
