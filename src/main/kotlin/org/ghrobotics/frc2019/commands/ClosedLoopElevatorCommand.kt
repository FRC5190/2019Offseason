/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.commands

import org.ghrobotics.frc2019.subsystems.Elevator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit

class ClosedLoopElevatorCommand(private val position: SIUnit<Meter>) :
  FalconCommand(Elevator) {
  override fun initialize() = Elevator.setPosition(position)
  override fun isFinished() = Elevator.atDesiredGoal
}
