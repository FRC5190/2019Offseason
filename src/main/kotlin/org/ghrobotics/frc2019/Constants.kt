/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Transform2d
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.lbs

object Constants {

  val kRobotMass = 145.lbs
  const val kRobotMomentOfInertia = 10.0 // kg m^2
  const val kRobotAngularDrag = 12.0 // Nm / (rad/s)

  val kRobotLength = 30.inches
  val kRobotWidth = 30.inches

  val kCenterToFrontCamera = Pose2d((-1.75).inches, 0.inches, Rotation2d())

  val kIntakeOffset = Transform2d(0.inches, 0.1.inches, Rotation2d())
  val kCenterToForwardIntake = Transform2d(16.inches, 0.inches, Rotation2d())

  const val kPCMId = 41
}
