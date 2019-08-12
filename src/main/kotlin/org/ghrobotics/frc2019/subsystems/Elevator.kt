/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.commands.DefaultElevatorCommand
import org.ghrobotics.frc2019.models.SpringCascadeElevatorModel
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable

object Elevator : FalconSubsystem(), EmergencyHandleable {
  private const val kMasterId: Int = 5
  private const val kSlave1Id: Int = 6
  private const val kSlave2Id: Int = 7
  private const val kSlave3Id: Int = 8

  private val kTransitionHeight = 9.inches
  private val kTransitionNativeUnit = 2490.nativeUnits
  private val kAfterTransitionSampleHeight = 54.5.inches
  private val kAfterTransitionNativeUnitSample = 9606.nativeUnits

  private const val kP = 1.0
  private val kBeforeTransitionKs = 1.3169999999999993.volts
  private val kBeforeTransitionKg = 0.7889999999999985.volts
  private val kAfterTransitionKs = 1.0709999999999904.volts
  private val kAfterTransitionKg = 1.5089999999999901.volts

  private val kPeakCurrentLimit = 40.amps
  private val kPeakCurrentLimitDuration = 250.milli.seconds
  private val kContinuousCurrentLimit = 20.amps

  private val kClosedLoopPositionTolerance = 1.inches
  private val kClosedLoopVelocityTolerance = 1.inches / 1.seconds

  private val kMotionProfileCruiseVelocity = 70.inches / 1.seconds
  private val kMotionProfileAcceleration = 122.5.inches / 1.seconds / 1.seconds

  private val kModel = SpringCascadeElevatorModel(
    transitionHeight = kTransitionHeight,
    transitionNativeUnit = kTransitionNativeUnit,
    afterTransitionSampleHeight = kAfterTransitionSampleHeight,
    afterTransitionSampleNativeUnit = kAfterTransitionNativeUnitSample
  )

  private val master = FalconSRX(kMasterId, kModel)
  private val slave1 = FalconSRX(kSlave1Id, kModel)
  private val slave2 = FalconSRX(kSlave2Id, kModel)
  private val slave3 = FalconSRX(kSlave3Id, kModel)

  private val allMotors = arrayOf(master, slave1, slave2, slave3)

  private val periodicIO = PeriodicIO()

  val height: SIUnit<Meter>
    get() = periodicIO.height

  val velocity: SIUnit<LinearVelocity>
    get() = periodicIO.velocity

  val atDesiredGoal: Boolean
    get() {
      val desiredOutput = periodicIO.desiredOutput
      if (desiredOutput is Output.Position) {
        return (height - desiredOutput.position).absoluteValue <
          kClosedLoopPositionTolerance && velocity.absoluteValue <
          kClosedLoopVelocityTolerance
      }
      return false
    }

  init {
    slave1.follow(master)
    slave2.follow(master)
    slave3.follow(master)

    with(master) {
      feedbackSensor = FeedbackDevice.QuadEncoder
      encoder.encoderPhase = false

      motionProfileCruiseVelocity = kMotionProfileCruiseVelocity
      motionProfileAcceleration = kMotionProfileAcceleration
      useMotionProfileForPosition = true

      with(talonSRX) {
        configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector,
          LimitSwitchNormal.NormallyOpen
        )
        configReverseLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector,
          LimitSwitchNormal.NormallyOpen
        )
        overrideLimitSwitchesEnable(true)
        configClearPositionOnLimitR(true, 10)

        configForwardSoftLimitThreshold(10850)
        configForwardSoftLimitEnable(true)

        setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20)
        configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms)

        selectProfileSlot(0, 0)
      }
    }

    allMotors.forEach { motor ->
      with(motor) {
        brakeMode = true
        voltageCompSaturation = 12.volts
        configCurrentLimit(
          true, FalconSRX.CurrentLimitConfig(
            kPeakCurrentLimit,
            kPeakCurrentLimitDuration,
            kContinuousCurrentLimit
          )
        )
      }
    }
    recoverFromEmergency()
    defaultCommand = DefaultElevatorCommand()
  }

  override fun activateEmergency() {
    master.talonSRX.config_kP(0, 0.0)
  }

  override fun recoverFromEmergency() {
    master.talonSRX.config_kP(0, kP)
  }

  override fun periodic() {
    periodicIO.voltage = master.voltageOutput
    periodicIO.current = master.talonSRX.outputCurrent.amps

    periodicIO.height = master.encoder.position
    periodicIO.velocity = master.encoder.velocity

    val feedforward = periodicIO.feedforward

    when (val desiredOutput = periodicIO.desiredOutput) {
      is Output.Nothing -> master.setNeutral()
      is Output.Percent -> master.setDutyCycle(
        desiredOutput.percent,
        feedforward
      )
      is Output.Position -> master.setPosition(
        desiredOutput.position,
        feedforward
      )
    }
  }

  override fun setNeutral() {
    periodicIO.desiredOutput = Output.Nothing
    periodicIO.feedforward = 0.volts
  }

  fun setPosition(position: SIUnit<Meter>) {
    periodicIO.desiredOutput = Output.Position(position)
    periodicIO.feedforward = calculateFF(position)
  }

  fun setPercent(percent: Double) {
    periodicIO.desiredOutput = Output.Percent(percent)
    periodicIO.feedforward = calculateFF(this.height)
  }

  private fun calculateFF(position: SIUnit<Meter>) =
    if (position < kTransitionHeight) {
      kBeforeTransitionKg + kBeforeTransitionKs
    } else {
      kAfterTransitionKg + kAfterTransitionKs
    }

  private class PeriodicIO {
    var voltage: SIUnit<Volt> = 0.volts
    var current: SIUnit<Ampere> = 0.amps

    var height: SIUnit<Meter> = 0.meters
    var velocity: SIUnit<LinearVelocity> = 0.meters / 1.seconds

    var desiredOutput: Output = Elevator.Output.Percent(0.0)
    var feedforward: SIUnit<Volt> = 0.volts
  }

  private sealed class Output {
    object Nothing : Output()
    class Percent(val percent: Double) : Output()
    class Position(val position: SIUnit<Meter>) : Output()
  }
}
