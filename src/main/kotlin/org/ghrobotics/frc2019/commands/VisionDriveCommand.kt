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
import org.ghrobotics.frc2019.subsystems.Elevator
import org.ghrobotics.frc2019.vision.LimelightManager
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.units.inFeet

class VisionDriveCommand(
  private val isFront: Boolean,
  private val disableLatencyComp: Boolean = false,
  private val autoPlace: Boolean = false
) : TeleopDriveCommand() {

  companion object {
    const val kLinearKp = 3.0 / 5.0
    const val kCorrectionKp = 0.8
    const val kCorrectionKd = 8.0
    var isActive = false
      private set
  }

  private var referencePose = Pose2d()
  private var lastKnownTargetPose: Pose2d? = null

  private var prevError = 0.0

  override fun initialize() {
    if (isFront) {
      LimelightManager.turnOnLED()
    }
    isActive = true
    referencePose = Drivetrain.robotPosition
  }

  @Suppress("ComplexMethod")
  override fun execute() {
    val newTarget =
      TargetTracker.getTargetUsingReference(referencePose, isFront)
    val newPose = newTarget?.averagePose

    if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose =
      newPose

    val lastKnownTargetPose = this.lastKnownTargetPose

    if (lastKnownTargetPose == null) {
      Elevator.requestVisionMode(true)
      super.execute()
    } else {
      Elevator.requestVisionMode(false)
      val transform =
        lastKnownTargetPose.relativeTo(
          Drivetrain.robotPosition + Constants.kIntakeOffset
        )

      val angle = if (disableLatencyComp) {
        Rotation2d(LimelightManager.frontLimelightAngleToTarget.value)
      } else {
        Rotation2d(transform.translation.x, transform.translation.y)
      }

      val source =
        if (autoPlace) kLinearKp * transform.translation.x_u.inFeet() / 17.0
        else -xSpeed()

      val error = angle +
        if (isFront) {
          Rotation2d()
        } else {
          Rotation2d.fromDegrees(180.0)
        }

      val turn =
        kCorrectionKp * error.radians * kCorrectionKd *
          (error.radians - prevError)
      prevError = error.radians

      Drivetrain.tankDrive(source - turn, source + turn)
    }
  }

  override fun end(interrupted: Boolean) {
    if (isFront) LimelightManager.turnOffLED()
    this.lastKnownTargetPose = null
    Elevator.requestVisionMode(false)
    isActive = false
  }
}
