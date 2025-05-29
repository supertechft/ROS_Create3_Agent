// robot/motion.js - Motion status, odometry, IMU


/**
 * Update all motion-related status displays
 * @param {Object} status - Full robot status object
 */
export function updateMotionStatus(status) {
  const { odometryStatus, imuStatus, stopStatus } = window.appElements;

  // Update dock status (imported from hazards.js to avoid circular dependency)
  import('./hazards.js').then(module => {
    module.updateDockStatus(status.dock_status);
  });

  // Update odometry, IMU, and stop status
  updateOdometryStatus(odometryStatus, status.odometry);
  updateImuStatus(imuStatus, status.imu);
  updateStopStatus(stopStatus, status.stop_status);
}


/**
 * Update odometry status display
 * @param {HTMLElement} container - Container element
 * @param {Object} odometry - Odometry data
 */
export function updateOdometryStatus(container, odometry) {
  container.innerHTML = '';
  if (odometry && Object.keys(odometry).length > 0) {
    // Position
    const positionDiv = document.createElement('div');
    positionDiv.className = 'odom-position';
    const position = odometry.position || {};
    positionDiv.textContent = `Position: x=${(position.x || 0).toFixed(2)}m, y=${(position.y || 0).toFixed(2)}m`;

    // Orientation (quaternion)
    const orientationDiv = document.createElement('div');
    orientationDiv.className = 'odom-orientation';
    const orientation = odometry.orientation || {};
    orientationDiv.textContent = `Orientation: [${(orientation.x || 0).toFixed(2)}, ${(orientation.y || 0).toFixed(2)}, ${(orientation.z || 0).toFixed(2)}, ${(orientation.w || 0).toFixed(2)}]`;

    // Velocity
    const velocityDiv = document.createElement('div');
    velocityDiv.className = 'odom-velocity';
    const linearVel = odometry.linear_velocity || {};
    const angularVel = odometry.angular_velocity || {};
    velocityDiv.textContent = `Speed: ${(linearVel.x || 0).toFixed(2)} m/s, Turn: ${(angularVel.z || 0).toFixed(2)} rad/s`;

    container.appendChild(positionDiv);
    container.appendChild(orientationDiv);
    container.appendChild(velocityDiv);
  } else {
    container.textContent = 'No data';
  }
}


/**
 * Update IMU status display
 * @param {HTMLElement} container - Container element
 * @param {Object} imu - IMU data
 */
export function updateImuStatus(container, imu) {
  container.innerHTML = '';
  if (imu && Object.keys(imu).length > 0) {
    // Orientation (simplified)
    const orientationDiv = document.createElement('div');
    orientationDiv.className = 'imu-orientation';
    const orientation = imu.orientation || {};
    orientationDiv.textContent = `Orientation: [${(orientation.x || 0).toFixed(2)}, ${(orientation.y || 0).toFixed(2)}, ${(orientation.z || 0).toFixed(2)}, ${(orientation.w || 0).toFixed(2)}]`;

    // Linear acceleration
    const accelDiv = document.createElement('div');
    accelDiv.className = 'imu-accel';
    const accel = imu.linear_acceleration || {};
    accelDiv.textContent = `Accel: [${(accel.x || 0).toFixed(2)}, ${(accel.y || 0).toFixed(2)}, ${(accel.z || 0).toFixed(2)}] m/s²`;

    // Angular velocity
    const gyroDiv = document.createElement('div');
    gyroDiv.className = 'imu-gyro';
    const gyro = imu.angular_velocity || {};
    gyroDiv.textContent = `Gyro: [${(gyro.x || 0).toFixed(2)}, ${(gyro.y || 0).toFixed(2)}, ${(gyro.z || 0).toFixed(2)}] rad/s`;

    container.appendChild(orientationDiv);
    container.appendChild(accelDiv);
    container.appendChild(gyroDiv);
  } else {
    container.textContent = 'No data';
  }
}


/**
 * Update stop status display
 * @param {HTMLElement} container - Container element
 * @param {Object} stopStatus - Stop status data
 */
export function updateStopStatus(container, stopStatus) {
  if (!stopStatus || stopStatus.is_stopped === null) {
    container.textContent = "Unknown";
    container.className = "status-value";
  } else if (stopStatus.is_stopped) {
    container.textContent = "Stopped";
    container.className = "status-value sensor-high";
  } else {
    container.textContent = "Moving";
    container.className = "status-value success-green";
  }
}
