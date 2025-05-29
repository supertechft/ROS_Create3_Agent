// robot/update.js - Main robot status module

import { updateBatteryStatus } from './battery.js';
import { updateHazardsStatus, updatePickedUpStatus } from './hazards.js';
import { updateMotionStatus } from './motion.js';
import { updateSensorReadings } from './sensors.js';


/**
 * Update robot status UI (battery, dock, hazards, sensors)
 * @param {Object} status - Robot status data from the server
 */
export function updateRobotStatus(status) {
  // Update battery status
  updateBatteryStatus(status.battery);


  // Update hazards status
  updateHazardsStatus(status.hazards);
  updatePickedUpStatus(status.is_picked_up);


  // Update motion status
  updateMotionStatus(status);


  // Update sensor readings
  const { proximityReadings, cliffReadings } = window.appElements;
  updateSensorReadings(proximityReadings, status.ir_intensities, false);
  updateSensorReadings(cliffReadings, status.cliff_intensities, true);
}
