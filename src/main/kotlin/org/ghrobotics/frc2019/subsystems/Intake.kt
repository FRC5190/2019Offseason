/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems

import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.AnalogInput
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSingleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake : FalconSubsystem() {
  private const val kMasterId: Int = 10
  private const val kSlave1Id: Int = 11

  private const val kExtensionSolenoidForwardId = 4
  private const val kExtensionSolenoidReverseId = 5
  private const val kLauncherSolenoidId = 3

  private const val kLeftBallSensorId = 0
  private const val kRightBallSensorId = 1

  private val kPeakCurrentLimit = 0.amps
  private val kPeakCurrentLimitDuration = 0.seconds
  private val kContinuousCurrentLimit = 25.amps

  private val master = FalconSRX(kMasterId, DefaultNativeUnitModel)
  private val slave1 = FalconSRX(kSlave1Id, DefaultNativeUnitModel)

  private val extensionSolenoid = FalconDoubleSolenoid(
    kExtensionSolenoidForwardId, kExtensionSolenoidReverseId, Constants.kPCMId
  )
  private val launcherSolenoid =
    FalconSingleSolenoid(kLauncherSolenoidId, Constants.kPCMId)

  private val leftBallSensor = AnalogInput(kLeftBallSensorId)
  private val rightBallSensor = AnalogInput(kRightBallSensorId)

  private val periodicIO = PeriodicIO()

  val pigeon = PigeonIMU(master.talonSRX)

  val isSeeingCargo get() = periodicIO.seeingCargo

  val extensionSolenoidState get() = extensionSolenoid.state
  val launcherSolenoidState get() = launcherSolenoid.state

  init {
    slave1.follow(master)

    master.outputInverted = true
    slave1.outputInverted = false

    arrayOf(master, slave1).forEach { motor ->
      motor.voltageCompSaturation = 12.volts
      motor.brakeMode = true
      motor.configCurrentLimit(
        true, FalconSRX.CurrentLimitConfig(
          kPeakCurrentLimit, kPeakCurrentLimitDuration, kContinuousCurrentLimit
        )
      )
    }

    pigeon.setTemperatureCompensationDisable(true)
  }

  override fun periodic() {
    periodicIO.voltage = master.voltageOutput
    periodicIO.current = master.talonSRX.outputCurrent.amps

    periodicIO.seeingCargo = leftBallSensor.averageVoltage > 1.7 ||
      rightBallSensor.averageVoltage > 1.2

    when (val desiredOutput = periodicIO.desiredOutput) {
      is Output.Nothing -> master.setNeutral()
      is Output.Percent -> master.setDutyCycle(desiredOutput.percent)
    }
  }

  override fun setNeutral() {
    periodicIO.desiredOutput = Output.Nothing
  }

  fun setPercent(percent: Double) {
    periodicIO.desiredOutput = Output.Percent(percent)
  }

  fun openIntake() {
    extensionSolenoid.state = FalconSolenoid.State.Forward
  }

  fun closeIntake() {
    extensionSolenoid.state = FalconSolenoid.State.Reverse
  }

  fun openLauncher() {
    launcherSolenoid.state = FalconSolenoid.State.Forward
  }

  fun closeLauncher() {
    launcherSolenoid.state = FalconSolenoid.State.Reverse
  }

  private class PeriodicIO {
    var voltage: SIUnit<Volt> = 0.volts
    var current: SIUnit<Ampere> = 0.amps

    var seeingCargo: Boolean = false

    var desiredOutput: Output = Output.Nothing
  }

  private sealed class Output {
    object Nothing : Output()
    class Percent(val percent: Double) : Output()
  }
}
