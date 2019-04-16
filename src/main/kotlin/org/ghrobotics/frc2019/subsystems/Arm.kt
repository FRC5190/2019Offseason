package org.ghrobotics.frc2019.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.lib.components.ArmComponent
import org.ghrobotics.lib.mathematics.threedim.geometry.Vector3
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.motors.ctre.FalconSRX

object Arm : ArmComponent(
    Vector3(0.0, 0.0, Constants.kElevatorSecondStageToArmShaft.value),
    Vector3(0.0, 1.0, 0.0)
) {

    override val motor = configMotor()

    override val armKg: Double get() = if (Intake.isHoldingHatch) Constants.kArmHatchKg else Constants.kArmEmptyKg

    // Debug Periodic Values
    var voltage = 0.0
        private set
    var current = 0.0
        private set
    var rawSensorPosition = 0.0
        private set

    override fun update() {
        super.update()

        if (Robot.shouldDebug) {
            voltage = motor.voltageOutput
            current = motor.talonSRX.outputCurrent
            rawSensorPosition = motor.encoder.rawPosition
        }
    }

    private fun configMotor(): FalconSRX<Rotation2d> {
        val armMaster = FalconSRX(Constants.kArmId, Constants.kArmNativeUnitModel)

        armMaster.outputInverted = true

        armMaster.feedbackSensor = FeedbackDevice.Analog
        armMaster.encoder.encoderPhase = false

        armMaster.brakeMode = true

        armMaster.voltageCompSaturation = 12.0

        armMaster.configCurrentLimit(
            true, FalconSRX.CurrentLimitConfig(
                0.amp,
                0.millisecond,
                Constants.kArmCurrentLimit
            )
        )

        armMaster.motionProfileCruiseVelocity = Constants.kArmCruiseVelocity.value
        armMaster.motionProfileAcceleration = Constants.kArmAcceleration.value
        armMaster.useMotionProfileForPosition = true

        armMaster.talonSRX.configForwardSoftLimitThreshold(Constants.kArmNativeUnitModel.toNativeUnitPosition(220.0).toInt())
        armMaster.talonSRX.configForwardSoftLimitEnable(false)

        armMaster.talonSRX.configReverseSoftLimitThreshold(Constants.kArmNativeUnitModel.toNativeUnitPosition(-40.0).toInt())
        armMaster.talonSRX.configReverseSoftLimitEnable(false)

        armMaster.talonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 0)
        armMaster.talonSRX.selectProfileSlot(0, 0)

        armMaster.talonSRX.config_kP(0, Constants.kArmKp)
        armMaster.talonSRX.config_kD(0, Constants.kArmKd)
        armMaster.talonSRX.config_kF(0, Constants.kArmKf)

        return armMaster
    }

}