/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.models

import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times

class ArmModel(
  sensorResolution: SIUnit<NativeUnit>,
  armVerticalTicks: SIUnit<NativeUnit>
) : NativeUnitModel<Radian>() {

  private val slope = sensorResolution / (2 * Math.PI).radians
  private val intercept = armVerticalTicks - ((Math.PI / 2).radians * slope)

  override fun fromNativeUnitPosition(nativeUnits: SIUnit<NativeUnit>):
    SIUnit<Radian> {
    return (nativeUnits - intercept) / slope
  }

  override fun toNativeUnitPosition(modelledUnit: SIUnit<Radian>):
    SIUnit<NativeUnit> {
    return slope * modelledUnit + intercept
  }
}
