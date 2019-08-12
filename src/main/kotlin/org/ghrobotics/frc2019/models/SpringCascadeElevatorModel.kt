/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.models

import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times

class SpringCascadeElevatorModel(
  private val transitionHeight: SIUnit<Meter>,
  private val transitionNativeUnit: SIUnit<NativeUnit>,
  afterTransitionSampleHeight: SIUnit<Meter>,
  afterTransitionSampleNativeUnit: SIUnit<NativeUnit>
) : NativeUnitModel<Meter>() {

  private val beforeTransitionSlope = transitionNativeUnit / transitionHeight
  private val afterTransitionSlope =
    (afterTransitionSampleNativeUnit - transitionNativeUnit) /
      (afterTransitionSampleHeight - transitionHeight)

  override fun fromNativeUnitPosition(nativeUnits: SIUnit<NativeUnit>) =
    when {
      nativeUnits < transitionNativeUnit -> nativeUnits / beforeTransitionSlope
      else -> {
        val afterTransitionNativeUnits = nativeUnits - transitionNativeUnit
        val afterTransitionHeight =
          afterTransitionNativeUnits / afterTransitionSlope
        transitionHeight + afterTransitionHeight
      }
    }

  override fun toNativeUnitPosition(modelledUnit: SIUnit<Meter>) =
    when {
      modelledUnit < transitionHeight -> beforeTransitionSlope * modelledUnit
      else -> {
        val afterTransitionHeight = modelledUnit - transitionHeight
        val afterTransitionNativeUnits =
          afterTransitionSlope * afterTransitionHeight
        transitionNativeUnit + afterTransitionNativeUnits
      }
    }

  override fun toNativeUnitError(modelledUnit: SIUnit<Meter>) =
    afterTransitionSlope * modelledUnit

  override fun fromNativeUnitVelocity(
    nativeUnitVelocity: SIUnit<NativeUnitVelocity>
  ): SIUnit<Velocity<Meter>> =
    SIUnit(nativeUnitVelocity.value / afterTransitionSlope.value)

  override fun toNativeUnitVelocity(
    modelledUnitVelocity: SIUnit<Velocity<Meter>>
  ): SIUnit<NativeUnitVelocity> =
    SIUnit(afterTransitionSlope.value * modelledUnitVelocity.value)
}
