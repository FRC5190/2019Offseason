/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.vision

import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.inches

object LimelightManager {

  private val kFrontLimelightHeight = 44.3.inches
  private val kFrontLimelightAngle = Rotation2d.fromDegrees(-30.0)
  private val kVisionTargetHeight = 29.inches

  private val notifier = Notifier(::update)

  private val frontLimelight = Limelight(
    kFrontLimelightHeight,
    kFrontLimelightAngle,
    kVisionTargetHeight,
    "limelight"
  )

  var frontLimelightAngleToTarget: SIUnit<Radian> = 0.degrees
    private set

  var isAlive: Boolean = false
    private set

  fun turnOnLED() = frontLimelight.turnOnLED()
  fun turnOffLED() = frontLimelight.turnOffLED()
  fun blinkLEDs() = frontLimelight.blinkLEDs()

  fun initialize() {
    notifier.startPeriodic(0.02)
  }

  private fun update() {
    frontLimelight.update()
    isAlive = frontLimelight.isAlive
    frontLimelightAngleToTarget = frontLimelight.angleToTarget
  }
}
