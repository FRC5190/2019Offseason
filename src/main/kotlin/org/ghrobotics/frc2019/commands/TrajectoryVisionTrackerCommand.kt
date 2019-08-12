/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.commands

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTrackerOutput
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.twodim.trajectory.Trajectory
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.utils.Source

class TrajectoryVisionTrackerCommand(
  private val trajectorySource: Source<Trajectory>,
  private val radiusFromEnd: SIUnit<Meter>,
  private val useAbsoluteVision: Boolean = false
) : FalconCommand(Drivetrain) {

  companion object {
    const val kCorrectionKp = 4.5
    var visionActive = false
  }

  private lateinit var trajectory: Trajectory
  private lateinit var lastPoint: Pose2d

  private var lastKnownTargetPose: Pose2d? = null

  override fun initialize() {
    trajectory = trajectorySource()
    lastPoint = trajectory.preview(1000.seconds).state.pose

    LiveDashboard.isFollowingPath = true
    Drivetrain.trajectoryTracker.reset(trajectory)
  }

  @Suppress("ComplexMethod")
  override fun execute() {
    val robotPose = Drivetrain.robotPosition
    val nextState = Drivetrain.trajectoryTracker.nextState(robotPose)

    val withinVisionRadius =
      Drivetrain.robotPosition.translation.getDistance(lastPoint.translation) <=
        radiusFromEnd.value

    if (withinVisionRadius) {
      val newTarget = if (!useAbsoluteVision) {
        TargetTracker.getBestTarget(!trajectory.reversed)
      } else {
        TargetTracker.getAbsoluteTarget(
          (lastPoint + Constants.kCenterToForwardIntake).translation
        )
      }
      val newPose = newTarget?.averagePose
      if (newTarget?.isAlive == true && newPose != null) {
        lastKnownTargetPose = newPose
      }
    }

    val lastKnownTargetPose = this.lastKnownTargetPose

    if (lastKnownTargetPose != null) {
      visionActive = true
      val transform =
        lastKnownTargetPose.relativeTo(robotPose + Constants.kIntakeOffset)
      val angle = Rotation2d(transform.translation.x, transform.translation.y)

      val error = angle +
        if (!trajectory.reversed) {
          Rotation2d()
        } else {
          Rotation2d.fromDegrees(180.0)
        }

      val turn = kCorrectionKp * error.radians

      Drivetrain.setOutput(
        TrajectoryTrackerOutput(
          nextState.linearVelocity,
          0.meters / 1.seconds / 1.seconds,
          turn.radians / 1.seconds,
          0.radians / 1.seconds / 1.seconds
        )
      )
    } else {
      Drivetrain.setOutput(nextState)
    }

    val referencePoint = Drivetrain.trajectoryTracker.referencePoint
    if (referencePoint != null) {
      val referencePose = referencePoint.state.pose

      // Update Current Path Location on Live Dashboard
      LiveDashboard.pathX = referencePose.translation.x_u.inFeet()
      LiveDashboard.pathY = referencePose.translation.y_u.inFeet()
      LiveDashboard.pathHeading = referencePose.rotation.radians
    }
  }

  override fun end(interrupted: Boolean) {
    Drivetrain.setNeutral()
    LiveDashboard.isFollowingPath = false
  }

  override fun isFinished(): Boolean = Drivetrain.trajectoryTracker.isFinished
}
