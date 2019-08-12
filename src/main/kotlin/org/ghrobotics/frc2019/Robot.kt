/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019

import org.ghrobotics.frc2019.subsystems.Arm
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.frc2019.subsystems.Elevator
import org.ghrobotics.frc2019.vision.LimelightManager
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {
  init {
    Controls

    +Drivetrain
    +Elevator
    +Arm
  }

  override fun robotInit() {
    LimelightManager.initialize()
  }

  @JvmStatic
  fun main(args: Array<String>) {
    start()
  }
}
