package org.ghrobotics.frc2019.subsystems.arm

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.intake.Intake
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnitsPer100ms
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable

object Arm : FalconSubsystem(), EmergencyHandleable, Loggable {

    private val masterMotor = FalconSRX(Constants.kArmId, Constants.kArmNativeUnitModel)

    @Log(name = "PeriodicIO")
    private val periodicIO = PeriodicIO()

    @Log.ToString(name = "Current State")
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    val angle: Rotation2d
        get() = Constants.kArmNativeUnitModel.fromNativeUnitPosition(periodicIO.rawSensorPosition.nativeUnits)

    val velocity: Velocity<Rotation2d>
        get() = Constants.kArmNativeUnitModel.fromNativeUnitVelocity(periodicIO.rawSensorVelocity.nativeUnitsPer100ms)


    init {
        masterMotor.apply {
            outputInverted = true

            feedbackSensor = FeedbackDevice.Analog
            encoder.encoderPhase = false

            brakeMode = true

            voltageCompSaturation = 12.0

            configCurrentLimit(
                true, FalconSRX.CurrentLimitConfig(
                    0.amp,
                    0.millisecond,
                    Constants.kArmCurrentLimit
                )
            )

            motionProfileCruiseVelocity = Constants.kArmCruiseVelocity.value
            motionProfileAcceleration = Constants.kArmAcceleration.value
            useMotionProfileForPosition = true

            talonSRX.configForwardSoftLimitThreshold(Constants.kArmNativeUnitModel.toNativeUnitPosition(220.0).toInt())
            talonSRX.configForwardSoftLimitEnable(false)

            talonSRX.configReverseSoftLimitThreshold(Constants.kArmNativeUnitModel.toNativeUnitPosition(-40.0).toInt())
            talonSRX.configReverseSoftLimitEnable(false)

            talonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 0)
            talonSRX.selectProfileSlot(0, 0)

            talonSRX.config_kP(0, Constants.kArmKp)
            talonSRX.config_kD(0, Constants.kArmKd)
            talonSRX.config_kF(0, Constants.kArmKf)
        }

        defaultCommand = DefaultArmCommand()
    }

    override fun activateEmergency() {
        masterMotor.talonSRX.config_kP(0, 0.0)
        masterMotor.talonSRX.config_kD(0, 0.0)
        masterMotor.talonSRX.config_kF(0, 0.0)
        zeroOutputs()
    }

    override fun recoverFromEmergency() {
        masterMotor.talonSRX.config_kP(0, Constants.kArmKp)
        masterMotor.talonSRX.config_kD(0, Constants.kArmKd)
        masterMotor.talonSRX.config_kF(0, Constants.kArmKf)
    }

    override fun periodic() {
        periodicIO.voltage = masterMotor.voltageOutput
        periodicIO.current = masterMotor.talonSRX.outputCurrent

        periodicIO.rawSensorPosition = masterMotor.encoder.rawPosition
        periodicIO.rawSensorVelocity = masterMotor.encoder.rawVelocity

        periodicIO.feedforward = Constants.kAccelerationDueToGravity * angle.cos *
            if (Intake.isHoldingHatch) Constants.kArmHatchKg else Constants.kArmEmptyKg

        when (wantedState) {
            State.Nothing -> masterMotor.setNeutral()
            State.MotionMagic -> masterMotor.setPosition(periodicIO.demand, periodicIO.feedforward)
            State.OpenLoop -> masterMotor.setDutyCycle(periodicIO.demand, periodicIO.feedforward)
        }

        if (currentState != wantedState) currentState = wantedState
    }

    fun setOpenLoop(percent: Double) {
        wantedState = State.OpenLoop
        periodicIO.demand = percent
    }

    fun setAngle(angle: Rotation2d) {
        wantedState = State.MotionMagic
        periodicIO.demand = angle.value
    }

    override fun zeroOutputs() {
        wantedState = State.Nothing
        periodicIO.demand = 0.0
    }

    private class PeriodicIO : Loggable {

        override fun configureLayoutType() = BuiltInLayouts.kGrid

        // Inputs
        @Log.VoltageView(name = "Voltage")
        var voltage: Double = 0.0

        @Log(name = "Current")
        var current: Double = 0.0

        @Log(name = "Raw Sensor Pos")
        var rawSensorPosition: Double = 0.0

        @Log(name = "Raw Sensor Vel")
        var rawSensorVelocity: Double = 0.0

        // Outputs
        var demand: Double = 0.0
        var feedforward: Double = 0.0
    }

    private enum class State {
        OpenLoop, MotionMagic, Nothing
    }
}