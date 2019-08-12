/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import org.ghrobotics.frc2019.commands.DefaultArmCommand
import org.ghrobotics.frc2019.models.ArmModel
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.ctre.FalconSRX
import kotlin.math.cos

object Arm : FalconSubsystem() {
  private const val kMasterId: Int = 9

  private val kSensorResolution = 1024.nativeUnits
  private val kVerticalTicks = (-495).nativeUnits

  private val kG = 1.5.volts

  private val kPeakCurrentLimit = 0.amps
  private val kPeakCurrentLimitDuration = 0.seconds
  private val kContinuousCurrentLimit = 15.amps

  private val kClosedLoopPositionTolerance = 5.degrees
  private val kClosedLoopVelocityTolerance = 2.degrees / 1.seconds

  private val kMotionProfileCruiseVelocity = 260.156.degrees / 1.seconds
  private val kMotionProfileAcceleration = 300.degrees / 1.seconds / 1.seconds

  private val kModel = ArmModel(kSensorResolution, kVerticalTicks)

  private val master = FalconSRX(kMasterId, kModel)

  private val periodicIO = PeriodicIO()

  val angle: SIUnit<Radian> get() = periodicIO.angle
  val velocity: SIUnit<AngularVelocity> get() = periodicIO.velocity

  val atDesiredGoal: Boolean
    get() {
      val desiredOutput = periodicIO.desiredOutput
      if (desiredOutput is Output.Position) {
        return (angle - desiredOutput.angle).absoluteValue <
          kClosedLoopPositionTolerance && velocity.absoluteValue <
          kClosedLoopVelocityTolerance
      }
      return false
    }

  init {
    with(master) {
      outputInverted = true

      feedbackSensor = FeedbackDevice.Analog
      encoder.encoderPhase = false

      brakeMode = true
      voltageCompSaturation = 12.volts

      motionProfileCruiseVelocity = kMotionProfileCruiseVelocity
      motionProfileAcceleration = kMotionProfileAcceleration
      useMotionProfileForPosition = true

      configCurrentLimit(
        true, FalconSRX.CurrentLimitConfig(
          kPeakCurrentLimit, kPeakCurrentLimitDuration, kContinuousCurrentLimit
        )
      )

      with(talonSRX) {
        configFeedbackNotContinuous(true, 10)
        setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20)
        selectProfileSlot(0, 0)
      }
    }
    defaultCommand = DefaultArmCommand()
  }

  override fun periodic() {
    periodicIO.voltage = master.voltageOutput
    periodicIO.current = master.talonSRX.outputCurrent.amps

    periodicIO.angle = master.encoder.position
    periodicIO.velocity = master.encoder.velocity

    val feedforward = periodicIO.feedforward

    when (val desiredOutput = periodicIO.desiredOutput) {
      is Nothing -> master.setNeutral()
      is Output.Percent -> master.setDutyCycle(
        desiredOutput.percent,
        feedforward
      )
      is Output.Position -> master.setPosition(desiredOutput.angle, feedforward)
    }
  }

  override fun setNeutral() {
    periodicIO.desiredOutput = Output.Nothing
    periodicIO.feedforward = 0.volts
  }

  fun setPosition(position: SIUnit<Radian>) {
    periodicIO.desiredOutput = Output.Position(position)
    periodicIO.feedforward = calculateFF(position)
  }

  fun setPercent(percent: Double) {
    periodicIO.desiredOutput = Output.Percent(percent)
    periodicIO.feedforward = calculateFF(this.angle)
  }

  private fun calculateFF(position: SIUnit<Radian>) = kG * cos(position.value)

  private class PeriodicIO {
    var voltage: SIUnit<Volt> = 0.volts
    var current: SIUnit<Ampere> = 0.amps

    var angle: SIUnit<Radian> = 0.radians
    var velocity: SIUnit<AngularVelocity> = 0.radians / 1.seconds

    var desiredOutput: Arm.Output = Arm.Output.Percent(0.0)
    var feedforward: SIUnit<Volt> = 0.volts
  }

  private sealed class Output {
    object Nothing : Output()
    class Percent(val percent: Double) : Output()
    class Position(val angle: SIUnit<Radian>) : Output()
  }
}
