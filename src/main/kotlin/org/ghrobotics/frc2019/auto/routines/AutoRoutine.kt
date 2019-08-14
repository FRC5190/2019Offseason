/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.auto.routines

import edu.wpi.first.wpilibj.experimental.command.Command
import edu.wpi.first.wpilibj.experimental.command.InstantCommand
import edu.wpi.first.wpilibj.experimental.command.WaitCommand
import edu.wpi.first.wpilibj.geometry.Pose2d
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.commands.TrajectoryVisionTrackerCommand
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallelDeadline
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.mirror
import org.ghrobotics.lib.mathematics.twodim.trajectory.Trajectory
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
import java.sql.Time

abstract class AutoRoutine : Source<Command> {
  abstract val duration: SIUnit<Second>
  abstract val routine: Command

  override fun invoke(): Command = sequential {
    +InstantCommand(Runnable {
      println("[AutoRoutine] Starting routine...")
      Drivetrain.localization.reset(Autonomous.startingPosition().pose)
    })
    +routine
  }.interruptOn { Robot.emergencyActive }

  protected fun executeFor(time: SIUnit<Second>, command: FalconCommand) =
    parallelDeadline(WaitCommand(time.value)) {
      +command
    }

  protected fun followVisionAssistedTrajectory(
    originalTrajectory: Trajectory,
    pathMirrored: BooleanSource,
    radiusFromEnd: SIUnit<Meter>,
    useAbsoluteVision: Boolean = false
  ): FalconCommand = TrajectoryVisionTrackerCommand(
    pathMirrored.map(originalTrajectory.mirror(), originalTrajectory),
    radiusFromEnd,
    useAbsoluteVision
  )

  protected fun relocalize(
    position: Pose2d,
    forward: Boolean,
    pathMirrored: BooleanSource
  ) = InstantCommand(Runnable {
    val newPosition = Pose2d(
      pathMirrored.map(position.mirror(), position)().translation,
      Drivetrain.localization().rotation
    ) + if (forward) {
      Constants.kForwardIntakeToCenter
    } else {
      Constants.kBackwardIntakeToCenter
    }
    Drivetrain.localization.reset(newPosition)
  })
}
