package org.ghrobotics.frc2019.subsystems.elevator

import com.ctre.phoenix.motorcontrol.*
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnitsPer100ms
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable

object Elevator : FalconSubsystem(), EmergencyHandleable, Loggable {

    private val masterMotor = FalconSRX(Constants.kElevatorMasterId, Constants.kElevatorNativeUnitModel)

    @Log(name = "PeriodicIO")
    private val periodicIO = PeriodicIO()

    @Log.ToString(name = "Current State")
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    val height: Length
        get() = Constants.kElevatorNativeUnitModel.fromNativeUnitPosition(periodicIO.rawSensorPosition.nativeUnits)

    val velocity: Velocity<Length>
        get() = Constants.kElevatorNativeUnitModel.fromNativeUnitVelocity(periodicIO.rawSensorVelocity.nativeUnitsPer100ms)

    val isZeroed: Boolean
        get() = periodicIO.reverseLimitSwitch

    init {
        masterMotor.apply {
            feedbackSensor = FeedbackDevice.QuadEncoder
            encoder.encoderPhase = false

            talonSRX.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen
            )
            talonSRX.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen
            )
            talonSRX.overrideLimitSwitchesEnable(true)

            talonSRX.configClearPositionOnLimitR(true, 0)

            talonSRX.configForwardSoftLimitThreshold(10850)
            talonSRX.configForwardSoftLimitEnable(true)

            motionProfileCruiseVelocity = Constants.kElevatorCruiseVelocity.value
            motionProfileAcceleration = Constants.kElevatorAcceleration.value
            useMotionProfileForPosition = true

            talonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20)
            talonSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms)

            talonSRX.selectProfileSlot(0, 0)
        }

        val slave1Motor = FalconSRX(Constants.kElevatorSlave1Id, Constants.kElevatorNativeUnitModel)
        val slave2Motor = FalconSRX(Constants.kElevatorSlave2Id, Constants.kElevatorNativeUnitModel)
        val slave3Motor = FalconSRX(Constants.kElevatorSlave3Id, Constants.kElevatorNativeUnitModel)

        slave1Motor.follow(masterMotor)
        slave2Motor.follow(masterMotor)
        slave3Motor.follow(masterMotor)

        fun configMotor(motor: FalconSRX<Length>) {
            motor.brakeMode = true
            motor.voltageCompSaturation = 12.0
            motor.configCurrentLimit(
                true, FalconSRX.CurrentLimitConfig(
                    0.amp,
                    0.millisecond,
                    Constants.kElevatorCurrentLimit
                )
            )
        }

        configMotor(masterMotor)
        configMotor(slave1Motor)
        configMotor(slave2Motor)
        configMotor(slave3Motor)

        masterMotor.talonSRX.config_kP(0, Constants.kElevatorKp)
        masterMotor.talonSRX.config_kD(0, Constants.kElevatorKd)
        masterMotor.talonSRX.config_kF(0, Constants.kElevatorKf)

        defaultCommand = DefaultElevatorCommand()
    }

    override fun activateEmergency() {
        masterMotor.talonSRX.config_kP(0, 0.0)
        masterMotor.talonSRX.config_kD(0, 0.0)
        masterMotor.talonSRX.config_kF(0, 0.0)

        zeroOutputs()
    }

    override fun recoverFromEmergency() {
        masterMotor.talonSRX.config_kP(0, Constants.kElevatorKp)
        masterMotor.talonSRX.config_kD(0, Constants.kElevatorKd)
        masterMotor.talonSRX.config_kF(0, Constants.kElevatorKf)
    }

    override fun periodic() {
        periodicIO.voltage = masterMotor.voltageOutput
        periodicIO.current = masterMotor.talonSRX.outputCurrent

        periodicIO.rawSensorPosition = masterMotor.encoder.rawPosition
        periodicIO.rawSensorVelocity = masterMotor.encoder.rawVelocity

        periodicIO.reverseLimitSwitch = masterMotor.talonSRX.sensorCollection.isRevLimitSwitchClosed

        periodicIO.feedforward = if (height < Constants.kElevatorSwitchHeight) {
            Constants.kElevatorBelowSwitchKg
        } else {
            Constants.kElevatorAfterSwitchKg
        }

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

    fun setHeight(height: Length) {
        wantedState = State.MotionMagic
        periodicIO.demand = height.value
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

        @Log(name = "Rev Limit")
        var reverseLimitSwitch: Boolean = false

        // Outputs
        var demand: Double = 0.0
        var feedforward: Double = 0.0
    }

    private enum class State {
        OpenLoop, MotionMagic, Nothing
    }
}