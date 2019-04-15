package org.ghrobotics.frc2019.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.components.DriveComponent
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.motors.LinearFalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX

object Drivetrain : DriveComponent(Constants.kDriveElevation) {

    init {
        addComponent(Elevator)
    }

    override val leftMotor = configMotor(
        Constants.kDriveLeftMasterId,
        Constants.kDriveRightSlaveId,
        false
    )
    override val rightMotor = configMotor(
        Constants.kDriveRightMasterId,
        Constants.kDriveRightSlaveId,
        true
    )

    override val differentialDrive = Constants.kDriveModel
    override val trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

    override val localization = TankEncoderLocalization(
        TODO("Pigeon is on intake"), //IntakeSubsystem.pigeonSource
        { leftMotor.encoder.position },
        { rightMotor.encoder.position }
    )

    private val shifter = Solenoid(Constants.kPCMId, Constants.kDriveSolenoidId)

    var wantedLowGear = false
    var currentLowGear = true
        private set

    override fun update() {
        super.update()

        if (wantedLowGear != currentLowGear) {
            currentLowGear =
                wantedLowGear
            shifter.set(currentLowGear)
        }
    }

    private fun configMotor(masterId: Int, slaveId: Int, inverted: Boolean): LinearFalconMotor {
        val masterMotor = FalconSRX(masterId, Constants.kDriveNativeUnitModel)
        val slaveMotor = FalconSRX(slaveId, Constants.kDriveNativeUnitModel)

        slaveMotor.follow(masterMotor)

        masterMotor.outputInverted = inverted
        slaveMotor.outputInverted = inverted

        masterMotor.feedbackSensor = FeedbackDevice.QuadEncoder
        masterMotor.encoder.encoderPhase = Constants.kDriveSensorPhase
        masterMotor.encoder.resetPosition(0.0)

        fun configMotor(motor: FalconSRX<Length>) {
            motor.talonSRX.configPeakOutputForward(1.0)
            motor.talonSRX.configPeakOutputReverse(-1.0)

            motor.talonSRX.configNominalOutputForward(0.0)
            motor.talonSRX.configNominalOutputReverse(0.0)

            motor.brakeMode = true

            motor.configCurrentLimit(
                true, FalconSRX.CurrentLimitConfig(
                    80.amp,
                    1.second,
                    Constants.kDriveCurrentLimit
                )
            )
        }

        configMotor(masterMotor)
        configMotor(slaveMotor)

        masterMotor.talonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10)

        masterMotor.talonSRX.config_kP(0, Constants.kDriveKp)
        masterMotor.talonSRX.config_kD(0, Constants.kDriveKd)

        return masterMotor
    }

}