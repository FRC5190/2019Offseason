/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.commands

import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import org.ghrobotics.frc2019.subsystems.Intake
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.FalconSolenoid

val IntakeCloseCommand
  get() = InstantCommand(Runnable {
    Intake.closeIntake()
    Intake.closeLauncher()
  }, Intake)

class IntakeHatchCommand(val releasing: Boolean) : FalconCommand(Intake) {
  override fun initialize() {
    if (releasing) {
      Intake.openIntake()
      Intake.setPercent(1.0)
    } else {
      Intake.closeIntake()
      Intake.setPercent(-1.0)
    }
  }

  override fun end(interrupted: Boolean) {
    if (releasing) {
      Intake.closeIntake()
    }
    Intake.setNeutral()
  }
}

class IntakeCargoCommand(val releasing: Boolean) : FalconCommand(Intake) {
  private var sensedBall = 0L
  private var startTime = 0L

  override fun initialize() {
    sensedBall = 0L
    startTime = System.currentTimeMillis()

    Intake.closeLauncher()

    if (releasing) {
      Intake.setPercent(-1.0)
      Intake.closeIntake()
    } else {
      Intake.openIntake()
      Intake.setPercent(1.0)
    }
  }

  override fun execute() {
    if (releasing) {
      if (System.currentTimeMillis() - startTime > 75 &&
        Intake.launcherSolenoidState == FalconSolenoid.State.Reverse
      ) {
        Intake.openLauncher()
      }
    } else {
      if (Intake.isSeeingCargo && sensedBall == 0L &&
        System.currentTimeMillis() - startTime > 500
      ) {
        Intake.closeIntake()
        sensedBall = System.currentTimeMillis()
      }
    }
  }

  override fun end(interrupted: Boolean) {
    Intake.closeLauncher()
    Intake.closeIntake()
    Intake.setNeutral()
  }

  override fun isFinished(): Boolean {
    if (releasing) {
      return sensedBall != 0L && System.currentTimeMillis() - sensedBall > 1000
    }
    return false
  }
}
