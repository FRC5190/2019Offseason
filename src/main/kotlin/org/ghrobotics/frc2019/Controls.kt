/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019

import org.ghrobotics.lib.wrappers.hid.*

object Controls {
  val driverXbox = xboxController(0) {
    registerEmergencyMode()
  }
  val operatorXbox = xboxController(1) {
    registerEmergencyMode()
  }

  private fun FalconXboxBuilder.registerEmergencyMode() {
    button(kBack).changeOn { Robot.activateEmergency() }
    button(kStart).changeOn { Robot.recoverFromEmergency() }
  }
}
