/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.models

import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.junit.Assert
import org.junit.Test

class ArmModelTest {
  private val model = ArmModel(1024.nativeUnits, (-495).nativeUnits)

  @Test
  fun testVerticalToNativeUnit() {
    val nativeUnit = model.toNativeUnitPosition(90.degrees)
    Assert.assertEquals((-495).nativeUnits, nativeUnit)
  }

  @Test
  fun testVerticalFromNativeUnit() {
    val modelledUnit = model.fromNativeUnitPosition((-495).nativeUnits)
    Assert.assertEquals(90.degrees, modelledUnit)
  }

  @Test
  fun testHorizontalToNativeUnit() {
    val nativeUnit = model.toNativeUnitPosition(0.degrees)
    Assert.assertEquals((-751).nativeUnits, nativeUnit)
  }

  @Test
  fun testHorizontalFronMativeUnit() {
    val modelledUnit = model.fromNativeUnitPosition((-751).nativeUnits)
    Assert.assertEquals(0.degrees, modelledUnit)
  }
}
