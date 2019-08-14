/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.vision

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.wrappers.networktables.get
import kotlin.math.tan

class Limelight(
  private val limelightHeight: SIUnit<Meter>,
  limelightAngle: Rotation2d,
  private val targetHeight: SIUnit<Meter>,
  private val key: String
) {
  private val limelightTable = NetworkTableInstance.getDefault().getTable(key)
  private val limelightAngle = limelightAngle.radians

  private val tx = limelightTable["tx"]
  private val ty = limelightTable["ty"]
  private val tl = limelightTable["tl"]
  private val tv = limelightTable["tv"]

  init {
    turnOffLED()
  }

  var isAlive: Boolean = false
    private set

  var angleToTarget: SIUnit<Radian> = 0.degrees
    private set

  fun turnOnLED() {
    limelightTable["ledMode"].setNumber(3)
  }

  fun turnOffLED() {
    limelightTable["ledMode"].setNumber(1)
  }

  fun blinkLEDs() {
    limelightTable["ledMode"].setNumber(2)
  }

  fun update() {
    val latency: Double = tl.getDouble(0.0) + 11
    isAlive = latency > 11

    if (tv.getNumber(0.0) == 0.00) return

    // tx has a negative sign because the Limelight is CW positive.
    // We don't want that.
    val tx = -Math.toRadians(tx.getDouble(0.0))
    val ty = +Math.toRadians(ty.getDouble(0.0))

    angleToTarget = tx.radians

    val distanceToTarget =
      (targetHeight - limelightHeight) / tan(limelightAngle + ty)
    if (distanceToTarget < Constants.kRobotLength / 2.2) return

    val timestamp: Double = Timer.getFPGATimestamp() - latency / 1000.0
    val transform =
      Transform2d(Translation2d(distanceToTarget, Rotation2d(tx)), Rotation2d())

    val driveLocation = Drivetrain.localization[timestamp.seconds]
    val targetRelativeToRobot = Constants.kCenterToFrontCamera + transform
    TargetTracker.addSamples(
      timestamp, listOf(
        driveLocation.transformBy(
          Transform2d(
            targetRelativeToRobot.translation,
            targetRelativeToRobot.rotation
          )
        )
      )
    )
  }
}
