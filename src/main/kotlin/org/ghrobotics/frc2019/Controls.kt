/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.experimental.command.Command
import org.ghrobotics.frc2019.commands.*
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.frc2019.subsystems.Intake
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.hid.*
import kotlin.math.pow
import kotlin.math.withSign

object Controls {

  private val pov180: Source<Command> = {
    when {
      backModifier -> Superstructure.kBackHatchFromLoadingStation
      Intake.isSeeingCargo -> TODO()
      else -> Superstructure.kFrontHatchFromLoadingStation
    }
  }

  private val pov270: Source<Command> = {
    if (backModifier) Superstructure.kBackCargoIntake
    else Superstructure.kFrontCargoIntake
  }

  private var backModifier = false

  val driverXbox = xboxController(0) {
    registerEmergencyMode()

    // Vision Align
    button(kY).change(
      VisionDriveCommand(
        isFront = true,
        disableLatencyComp = true,
        autoPlace = true
      )
    )
    button(kB).change(VisionDriveCommand(isFront = false))

    // Shifting
    button(kA).changeOn { Drivetrain.setLowGear() }
    button(kA).changeOff { Drivetrain.setHighGear() }

    // Intake
    triggerAxisButton(GenericHID.Hand.kLeft).change(IntakeHatchCommand(true))
    button(kBumperLeft).change(IntakeHatchCommand(false))
    triggerAxisButton(GenericHID.Hand.kRight).change(IntakeCargoCommand(true))
    button(kBumperRight).change(IntakeCargoCommand(false))
  }

  val operatorXbox = xboxController(1) {
    registerEmergencyMode()

    // Back Modifier
    triggerAxisButton(GenericHID.Hand.kLeft, 0.20) {
      changeOn { backModifier = true }
      changeOff { backModifier = false }
    }

    // Manual Control
    axisButton(
      axisId = 1,
      threshold = 0.1
    ) {
      change(
        OpenLoopElevatorCommand(
          source.map { it.pow(2).withSign(-it) * .5 })
      )
    }
    axisButton(
      axisId = 5,
      threshold = 0.1
    ) {
      change(
        OpenLoopArmCommand(
          source.map { it.pow(2).withSign(-it) * .5 })
      )
    }

    // Presets
    pov(180).changeOn { pov180().schedule() }
    pov(270).changeOn { pov270().schedule() }
  }

  private fun FalconXboxBuilder.registerEmergencyMode() {
    button(kBack).changeOn { Robot.activateEmergency() }
    button(kStart).changeOn { Robot.recoverFromEmergency() }
  }
}
