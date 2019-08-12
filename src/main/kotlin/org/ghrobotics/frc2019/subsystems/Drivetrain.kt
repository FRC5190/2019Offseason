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
import com.ctre.phoenix.sensors.PigeonIMU
import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.commands.TeleopDriveCommand
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.mathematics.units.operations.times
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import org.ghrobotics.lib.utils.asSource
import kotlin.math.pow

object Drivetrain : TankDriveSubsystem(), EmergencyHandleable {

  private const val kLeftMasterId = 1
  private const val kLeftSlave1Id = 2
  private const val kRightMasterId = 3
  private const val kRightSlave1Id = 4
  private const val kPigeonId = 17
  private const val kShifterId = 0

  private const val kBeta = 2.0
  private const val kZeta = 0.7

  private val kWheelRadius = 3.inches
  private val kTrackWidth = 29.inches

  private const val kP = 2.35
  private const val kD = 2.0
  private const val kLeftKv = 0.1489
  private const val kLeftKa = 0.05
  private const val kLeftKs = 1.2423
  private const val kRightKv = 0.1475
  private const val kRightKa = 0.05
  private const val kRightKs = 1.2468

  private val kLeftTransmission = DCMotorTransmission(
    1 / kLeftKv,
    kWheelRadius.value.pow(2) * Constants.kRobotMass.value / (2.0 * kLeftKa),
    kLeftKs
  )

  private val kRightTransmission = DCMotorTransmission(
    1 / kRightKv,
    kWheelRadius.value.pow(2) * Constants.kRobotMass.value / (2 * kRightKa),
    kRightKs
  )

  private val kPeakCurrentLimit = 70.amps
  private val kPeakCurrentLimitDuration = 700.milli.seconds
  private val kContinuousCurrentLimit = 38.amps

  private val kModel = SlopeNativeUnitModel(138.inches, 10000.nativeUnits)

  override val leftMotor = FalconSRX(kLeftMasterId, kModel)
  private val leftSlave1 = FalconSRX(kLeftSlave1Id, kModel)
  override val rightMotor = FalconSRX(kRightMasterId, kModel)
  private val rightSlave1 = FalconSRX(kRightSlave1Id, kModel)
  private val gyro = PigeonIMU(kPigeonId)

  private val shifter = Solenoid(Constants.kPCMId, kShifterId)

  private val allMasters = arrayOf(leftMotor, rightMotor)
  private val allMotors = allMasters + arrayOf(leftSlave1, rightSlave1)

  override val differentialDrive = DifferentialDrive(
    Constants.kRobotMass.value,
    Constants.kRobotMomentOfInertia,
    Constants.kRobotAngularDrag,
    kWheelRadius.value,
    kTrackWidth.value / 2,
    kLeftTransmission,
    kRightTransmission
  )

  override val localization = TankEncoderLocalization(
    gyro.asSource(),
    ::leftPosition, ::rightPosition
  )

  override val trajectoryTracker = RamseteTracker(kBeta, kZeta)

  private val periodicIO = PeriodicIO()

  val leftPosition: SIUnit<Meter> = periodicIO.leftPosition
  val rightPosition: SIUnit<Meter> = periodicIO.rightPosition

  init {
    leftSlave1.follow(leftMotor)
    rightSlave1.follow(rightMotor)

    leftMotor.outputInverted = false
    leftSlave1.outputInverted = false
    rightMotor.outputInverted = true
    rightSlave1.outputInverted = true

    allMasters.forEach { master ->
      master.feedbackSensor = FeedbackDevice.QuadEncoder
      master.encoder.encoderPhase = true
      master.talonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10)
    }

    allMotors.forEach { motor ->
      motor.brakeMode = true
      motor.voltageCompSaturation = 12.volts

      motor.configCurrentLimit(
        true, FalconSRX.CurrentLimitConfig(
          kPeakCurrentLimit, kPeakCurrentLimitDuration, kContinuousCurrentLimit
        )
      )
    }
    recoverFromEmergency()
    defaultCommand = TeleopDriveCommand()
  }

  override fun activateEmergency() {
    allMasters.forEach { master ->
      master.talonSRX.config_kP(0, 0.0)
      master.talonSRX.config_kD(0, 0.0)
    }
  }

  override fun recoverFromEmergency() {
    allMasters.forEach { master ->
      master.talonSRX.config_kP(0, kP)
      master.talonSRX.config_kD(0, kD)
    }
  }

  override fun periodic() {
    periodicIO.leftVoltage = leftMotor.voltageOutput
    periodicIO.rightVoltage = rightMotor.voltageOutput

    periodicIO.leftCurrent = leftMotor.talonSRX.outputCurrent.amps
    periodicIO.rightCurrent = rightMotor.talonSRX.outputCurrent.amps

    periodicIO.leftPosition = leftMotor.encoder.position
    periodicIO.rightPosition = rightMotor.encoder.position

    val leftFeedforward = periodicIO.leftFeedforward
    val rightFeedforward = periodicIO.rightFeedforward

    when (val desiredOutput = periodicIO.desiredOutput) {
      is Output.Nothing -> {
        leftMotor.setNeutral()
        rightMotor.setNeutral()
      }
      is Output.Percent -> {
        leftMotor.setDutyCycle(desiredOutput.left, leftFeedforward)
        rightMotor.setDutyCycle(desiredOutput.right, rightFeedforward)
      }
      is Output.Velocity -> {
        leftMotor.setVelocity(desiredOutput.left, leftFeedforward)
        rightMotor.setVelocity(desiredOutput.right, rightFeedforward)
      }
    }
  }

  override fun setOutput(
    wheelVelocities: DifferentialDrive.WheelState,
    wheelVoltages: DifferentialDrive.WheelState
  ) {
    periodicIO.desiredOutput = Output.Velocity(
      kWheelRadius * wheelVelocities.left.radians / 1.seconds,
      kWheelRadius * wheelVelocities.right.radians / 1.seconds
    )
    periodicIO.leftFeedforward = wheelVoltages.left.volts
    periodicIO.rightFeedforward = wheelVoltages.right.volts
  }

  override fun tankDrive(leftPercent: Double, rightPercent: Double) =
    setPercent(leftPercent, rightPercent)

  override fun setNeutral() {
    periodicIO.desiredOutput = Output.Nothing
    periodicIO.leftFeedforward = 0.volts
    periodicIO.rightFeedforward = 0.volts
  }

  fun setPercent(left: Double, right: Double) {
    periodicIO.desiredOutput = Output.Percent(left, right)
    periodicIO.leftFeedforward = 0.volts
    periodicIO.rightFeedforward = 0.volts
  }

  private class PeriodicIO {
    var leftVoltage: SIUnit<Volt> = 0.volts
    var rightVoltage: SIUnit<Volt> = 0.volts

    var leftCurrent: SIUnit<Ampere> = 0.amps
    var rightCurrent: SIUnit<Ampere> = 0.amps

    var leftPosition: SIUnit<Meter> = 0.meters
    var rightPosition: SIUnit<Meter> = 0.meters

    var desiredOutput: Output = Output.Nothing
    var leftFeedforward: SIUnit<Volt> = 0.volts
    var rightFeedforward: SIUnit<Volt> = 0.volts
  }

  private sealed class Output {
    object Nothing : Output()
    class Percent(val left: Double, val right: Double) : Output()
    class Velocity(
      val left: SIUnit<LinearVelocity>,
      val right: SIUnit<LinearVelocity>
    ) : Output()
  }

}
