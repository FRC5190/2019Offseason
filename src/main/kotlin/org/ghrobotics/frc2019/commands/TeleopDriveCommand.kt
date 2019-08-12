/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.commands

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kA

open class TeleopDriveCommand : FalconCommand(Drivetrain) {
  companion object {
    val xSpeed = Controls.driverXbox.getY(GenericHID.Hand.kLeft)
    val ySpeed = Controls.driverXbox.getX(GenericHID.Hand.kLeft)
    val quickTurn = Controls.driverXbox.getRawButton(kA)
  }

  override fun execute() =
    Drivetrain.curvatureDrive(-xSpeed(), ySpeed(), quickTurn())
}
