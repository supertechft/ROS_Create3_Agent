// robot/hazards.js - Hazard detection display


/**
 * Update hazards list display
 * @param {Array} hazards - List of hazard objects
 */
export function updateHazardsStatus(hazards) {
  const hazardsList = window.appElements.hazardsList;

  hazardsList.innerHTML = '';
  if (hazards && hazards.length > 0) {
    hazards.forEach(hazard => {
      const hazardElement = document.createElement('div');

      // Classify hazard type for styling
      let hazardType = 'unknown';
      if (hazard.type.includes('BUMP')) hazardType = 'bump';
      else if (hazard.type.includes('CLIFF')) hazardType = 'cliff';
      else if (hazard.type.includes('WHEEL_DROP')) hazardType = 'wheel-drop';
      else if (hazard.type.includes('BACKUP_LIMIT')) hazardType = 'backup-limit';
      else if (hazard.type.includes('OBJECT_PROXIMITY')) hazardType = 'proximity';
      hazardElement.className = `hazard-item hazard-${hazardType}`;
      hazardElement.textContent = hazard.description;

      // Add location if available
      if (hazard.location) {
        const locationSpan = document.createElement('span');
        locationSpan.className = 'hazard-location';
        locationSpan.textContent = ` (${hazard.location})`;
        hazardElement.appendChild(locationSpan);
      }
      hazardsList.appendChild(hazardElement);
    });
  } else {
    hazardsList.textContent = 'None';
  }
}


/**
 * Update picked up status display
 * @param {boolean} isPickedUp - Whether the robot is picked up
 */
export function updatePickedUpStatus(isPickedUp) {
  const pickedUpStatus = window.appElements.pickedUpStatus;

  if (isPickedUp === null) {
    pickedUpStatus.textContent = 'Unknown';
    pickedUpStatus.className = '';
  } else {
    pickedUpStatus.textContent = isPickedUp ? 'Yes' : 'No';
    pickedUpStatus.className = isPickedUp ? 'picked-up-true' : 'picked-up-false';
  }
}


/**
 * Update dock status display
 * @param {Object} dockStatusInfo - Dock status information
 */
export function updateDockStatus(dockStatusInfo) {
  const dockStatusElement = document.getElementById('dock-status');
  const dockVisibleElement = document.getElementById('dock-visible');

  const dockData = dockStatusInfo || {};
  const isDocked = dockData.is_docked;
  const isVisible = dockData.dock_visible;

  // Update docked status (Yes/No like picked-up-status)
  if (isDocked === null || isDocked === undefined) {
    dockStatusElement.textContent = 'Unknown';
    dockStatusElement.className = 'status-value';
  } else if (isDocked) {
    dockStatusElement.textContent = 'Yes';
    dockStatusElement.className = 'status-value picked-up-true';
  } else {
    dockStatusElement.textContent = 'No';
    dockStatusElement.className = 'status-value picked-up-false';
  }

  // Update station visible status (Yes/No)
  if (isVisible === null || isVisible === undefined) {
    dockVisibleElement.textContent = 'Unknown';
    dockVisibleElement.className = 'status-value';
  } else if (isVisible) {
    dockVisibleElement.textContent = 'Yes';
    dockVisibleElement.className = 'status-value picked-up-true';
  } else {
    dockVisibleElement.textContent = 'No';
    dockVisibleElement.className = 'status-value picked-up-false';
  }
}
