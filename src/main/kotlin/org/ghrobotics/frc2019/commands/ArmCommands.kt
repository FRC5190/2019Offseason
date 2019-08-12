/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.commands

import org.ghrobotics.frc2019.subsystems.Arm
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.utils.DoubleSource

class DefaultArmCommand : FalconCommand(Arm) {
  override fun initialize() = Arm.setPosition(Arm.angle)
}

class OpenLoopArmCommand(private val percent: DoubleSource) :
  FalconCommand(Arm) {
  override fun execute() = Arm.setPercent(percent())
}

class ClosedLoopArmCommand(private val angle: SIUnit<Radian>) :
  FalconCommand(Arm) {
  override fun initialize() = Arm.setPosition(angle)
  override fun isFinished() = Arm.atDesiredGoal
}
