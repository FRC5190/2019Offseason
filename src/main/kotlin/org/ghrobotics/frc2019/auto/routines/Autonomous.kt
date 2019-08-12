/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.auto.routines

import edu.wpi.first.wpilibj.geometry.Pose2d
import org.ghrobotics.frc2019.auto.Waypoints
import org.ghrobotics.lib.mathematics.twodim.geometry.mirror

object Autonomous {

  val startingPosition = { StartingPositions.kLeft }

  enum class StartingPositions(val pose: Pose2d) {
    kLeft(Waypoints.kSideStart.mirror()),
    kCenter(Waypoints.kCenterStart),
    kRight(Waypoints.kSideStart),
    kLeftReversed(Waypoints.kSideStartReversed.mirror()),
    kRightReversed(Waypoints.kSideStartReversed)
  }
}
