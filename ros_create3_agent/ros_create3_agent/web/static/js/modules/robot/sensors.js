// robot/sensors.js - Sensor readings display


/**
 * Update proximity or cliff sensor readings
 * @param {HTMLElement} container - Container element
 * @param {Object} readings - Sensor readings data
 * @param {boolean} isCliff - Whether these are cliff sensors (vs proximity)
 */
export function updateSensorReadings(container, readings, isCliff) {
  container.innerHTML = '';
  if (readings && Object.keys(readings).length > 0) {
    for (const [sensorName, value] of Object.entries(readings)) {
      const readingElement = document.createElement('div');
      readingElement.className = 'sensor-reading';

      // Color code based on value and sensor type
      if (isCliff) {
        if (value < 20) readingElement.classList.add('sensor-reading-high');
        else if (value < 50) readingElement.classList.add('sensor-reading-medium');
        else readingElement.classList.add('sensor-reading-low');
      } else {
        if (value > 50) readingElement.classList.add('sensor-reading-high');
        else if (value > 20) readingElement.classList.add('sensor-reading-medium');
        else readingElement.classList.add('sensor-reading-low');
      }

      readingElement.textContent = `${sensorName}: ${value}`;
      container.appendChild(readingElement);
    }
  } else {
    container.textContent = 'No data';
  }
}
