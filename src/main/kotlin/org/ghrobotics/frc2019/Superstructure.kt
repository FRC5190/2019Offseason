/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.experimental.command.Command
import edu.wpi.first.wpilibj.experimental.command.ConditionalCommand
import edu.wpi.first.wpilibj.experimental.command.RunCommand
import edu.wpi.first.wpilibj.experimental.command.WaitUntilCommand
import org.ghrobotics.frc2019.commands.ClosedLoopArmCommand
import org.ghrobotics.frc2019.commands.ClosedLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.Arm
import org.ghrobotics.frc2019.subsystems.Intake
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.utils.BooleanSource
import java.util.function.BooleanSupplier
import kotlin.math.cos
import kotlin.math.sign

object Superstructure {

  private val kArmFlipTolerance = 30.degrees

  val kFrontHatchFromLoadingStation
    get() = setElevatorAndArmHeight(
      1.inches,
      10.degrees
    )
  val kBackHatchFromLoadingStation
    get() = setElevatorAndArmHeight(
      1.inches,
      173.degrees
    )

  val kFrontCargoIntake get() = setElevatorAndArmHeight(0.inches, (-25).degrees)
  val kBackCargoIntake get() = setElevatorAndArmHeight(0.inches, (-155).degrees)

  val kFrontCargoIntoCargoShip
    get() = setElevatorAndArmHeight(
      25.inches,
      5.degrees
    )

  val kStowedPosition get() = setElevatorAndArmHeight(0.inches, 75.degrees)

  fun setElevatorAndArmHeight(
    elevatorHeight: SIUnit<Meter>,
    armAngle: SIUnit<Radian>
  ): Command {

    val isArmInBack: Boolean = cos(Arm.angle.value) < 0
    val passthrough: Boolean =
      sign(cos(Arm.angle.value)) != sign(cos(armAngle.value))

    val armSafePosition: SIUnit<Radian> =
      90.degrees + if (isArmInBack) kArmFlipTolerance else -kArmFlipTolerance

    val armCleared: BooleanSource = {
      if (isArmInBack) {
        cos(Arm.angle.value) > 1 && Arm.angle < 90.degrees - kArmFlipTolerance
      } else {
        cos(Arm.angle.value) < 1 && Arm.angle > 90.degrees + kArmFlipTolerance
      }
    }

    return ConditionalCommand(
      // On True
      sequential {
        // First, bring elevator down and arm to safe position
        +parallelDeadline(ClosedLoopElevatorCommand((-3).inches)) {
          +ClosedLoopArmCommand(armSafePosition)
        }
        // Then, flip and go to final position
        +parallel {
          // Get the arm to the final position
          +ClosedLoopArmCommand(armAngle)
          +sequential {
            // First, close the intake while flipping
            +parallelDeadline(WaitUntilCommand(armCleared)) {
              +RunCommand(Runnable { Intake.closeIntake() })
            }
            // Then, take the elevator up when the arm is clear
            +ClosedLoopElevatorCommand(elevatorHeight)
          }
        }
      },
      // On False
      sequential {
        +ClosedLoopElevatorCommand(elevatorHeight)
        +ClosedLoopArmCommand(armAngle)
      },
      BooleanSupplier { passthrough }
    )
  }
}
