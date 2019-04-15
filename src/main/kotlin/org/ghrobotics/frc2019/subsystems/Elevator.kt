package org.ghrobotics.frc2019.subsystems

import com.ctre.phoenix.motorcontrol.*
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.components.SpringCascadeElevatorComponent
import org.ghrobotics.lib.mathematics.threedim.geometry.Vector3
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.motors.ctre.FalconSRX

object Elevator : SpringCascadeElevatorComponent(
    Vector3(0.0, 0.0, Constants.kElevatorHeightFromDrivetrain.value),
    Constants.kElevatorSwitchHeight.value,
    Constants.kElevatorAfterSwitchKg,
    Constants.kElevatorBelowSwitchKg
) {

    override val motor = configMotor()

    // Wanted States
    var wantedVisionMode = false

    // Periodic Values
    var isBottomLimitSwitchPressed = false
        private set
    var isZeroed = false
        private set

    // Debug Periodic Values
    var current = 0.0
        private set
    var rawSensorPosition = 0.0
        private set
    var voltage = 0.0
        private set

    override fun customizeWantedState(wantedState: State): State {
        // Move carriage up when aligning to vision target (if carriage is blocking the camera)
        return if (wantedVisionMode && wantedState is State.Position && wantedState.position in Constants.kElevatorBlockingCameraRange) {
            State.Position(Constants.kElevatorVisionPosition.value)
        } else {
            super.customizeWantedState(wantedState)
        }
    }

    override fun update() {
        super.update()

        isBottomLimitSwitchPressed = motor.talonSRX.sensorCollection.isRevLimitSwitchClosed
        if (isBottomLimitSwitchPressed) isZeroed = true

        if (Robot.shouldDebug) {
            current = motor.talonSRX.outputCurrent
            voltage = motor.voltageOutput
            rawSensorPosition = motor.encoder.rawPosition
        }
    }

    private fun configMotor(): FalconSRX<Length> {
        val masterMotor = FalconSRX(Constants.kElevatorMasterId, Constants.kElevatorNativeUnitModel)
        val slave1Motor = FalconSRX(Constants.kElevatorSlave1Id, Constants.kElevatorNativeUnitModel)
        val slave2Motor = FalconSRX(Constants.kElevatorSlave2Id, Constants.kElevatorNativeUnitModel)
        val slave3Motor = FalconSRX(Constants.kElevatorSlave3Id, Constants.kElevatorNativeUnitModel)

        slave1Motor.follow(masterMotor)
        slave2Motor.follow(masterMotor)
        slave3Motor.follow(masterMotor)

        masterMotor.feedbackSensor = FeedbackDevice.QuadEncoder
        masterMotor.encoder.encoderPhase = false

        masterMotor.talonSRX.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen
        )
        masterMotor.talonSRX.configReverseLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen
        )
        masterMotor.talonSRX.overrideLimitSwitchesEnable(true)

        masterMotor.talonSRX.configClearPositionOnLimitR(true, 0)

        masterMotor.talonSRX.configForwardSoftLimitThreshold(10850)
        masterMotor.talonSRX.configForwardSoftLimitEnable(true)

        masterMotor.motionProfileCruiseVelocity = Constants.kElevatorCruiseVelocity.value
        masterMotor.motionProfileAcceleration = Constants.kElevatorAcceleration.value

        masterMotor.talonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20)
        masterMotor.talonSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms)

        masterMotor.talonSRX.selectProfileSlot(0, 0)

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

        return masterMotor
    }

}