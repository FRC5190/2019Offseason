package org.ghrobotics.frc2019.subsystems.drivetrain

import asSource
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.sensors.PigeonIMU
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.EmergencyHandleable
import org.ghrobotics.frc2019.subsystems.intake.Intake
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import kotlin.properties.Delegates

object Drivetrain : TankDriveSubsystem(), EmergencyHandleable {

    override val leftMotor = configureDriveGearbox(Constants.kDriveLeftMasterId, Constants.kDriveLeftSlaveId, false)
    override val rightMotor = configureDriveGearbox(Constants.kDriveRightMasterId, Constants.kDriveRightSlaveId, true)
    private var wantedState = State.Nothing

    override val differentialDrive = Constants.kDriveModel
    override val trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

    private val shifter = Solenoid(Constants.kPCMId, Constants.kDriveSolenoidId)

    override val localization = TankEncoderLocalization(
        PigeonIMU(Intake.masterMotor).asSource(),
        { lPosition.value },
        { rPosition.value }
    )

    val lPosition: Length
        get() = Constants.kDriveNativeUnitModel.fromNativeUnitPosition(PeriodicIO.leftRawSensorPosition.nativeUnits)

    val rPosition: Length
        get() = Constants.kDriveNativeUnitModel.fromNativeUnitPosition(PeriodicIO.rightRawSensorPosition.nativeUnits)

    var lowGear by Delegates.observable(false) { _, _, wantLow ->
        if (wantLow) {
            shifter.set(true)
        } else {
            shifter.set(false)
        }
    }

    init {
        Notifier(localization::update).startPeriodic(1.0 / 100.0)
        defaultCommand = TeleopDriveCommand()
    }

    private fun configureDriveGearbox(masterId: Int, slaveId: Int, inverted: Boolean): FalconSRX<Length> {
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

    override fun activateEmergency() {
        listOf(leftMotor, rightMotor).forEach { masterMotor ->
            masterMotor.talonSRX.config_kP(0, 0.0)
            masterMotor.talonSRX.config_kD(0, 0.0)
        }
        zeroOutputs()
    }

    override fun recoverFromEmergency() {
        listOf(leftMotor, rightMotor).forEach { masterMotor ->
            masterMotor.talonSRX.config_kP(0, Constants.kDriveKp)
            masterMotor.talonSRX.config_kD(0, Constants.kDriveKd)
        }
    }

    override fun periodic() {
        PeriodicIO.leftVoltage = leftMotor.voltageOutput
        PeriodicIO.rightVoltage = rightMotor.voltageOutput

        PeriodicIO.leftCurrent = leftMotor.talonSRX.outputCurrent
        PeriodicIO.rightCurrent = rightMotor.talonSRX.outputCurrent

        PeriodicIO.leftRawSensorPosition = leftMotor.encoder.rawPosition
        PeriodicIO.rightRawSensorPosition = rightMotor.encoder.rawPosition

        PeriodicIO.leftRawSensorVelocity = leftMotor.encoder.rawVelocity
        PeriodicIO.rightRawSensorVelocity = rightMotor.encoder.rawVelocity

        when (wantedState) {
            State.Nothing -> {
                leftMotor.setNeutral()
                rightMotor.setNeutral()
            }
            State.PathFollowing -> {
                leftMotor.setVelocity(PeriodicIO.leftDemand, PeriodicIO.leftFeedforward)
                rightMotor.setVelocity(PeriodicIO.rightDemand, PeriodicIO.rightFeedforward)
            }
            State.OpenLoop -> {
                leftMotor.setDutyCycle(PeriodicIO.leftDemand)
                rightMotor.setDutyCycle(PeriodicIO.rightDemand)
            }
        }
    }

    override fun tankDrive(leftPercent: Double, rightPercent: Double) = setOpenLoop(leftPercent, rightPercent)

    fun setOpenLoop(left: Double, right: Double) {
        wantedState = State.OpenLoop

        PeriodicIO.leftDemand = left
        PeriodicIO.rightDemand = right

        PeriodicIO.leftFeedforward = 0.0
        PeriodicIO.rightFeedforward
    }


    override fun setOutput(wheelVelocities: DifferentialDrive.WheelState, wheelVoltages: DifferentialDrive.WheelState) {
        wantedState = State.PathFollowing

        PeriodicIO.leftDemand = wheelVelocities.left * differentialDrive.wheelRadius
        PeriodicIO.rightDemand = wheelVelocities.right * differentialDrive.wheelRadius

        PeriodicIO.leftFeedforward = wheelVoltages.left
        PeriodicIO.rightFeedforward = wheelVoltages.right
    }

    override fun zeroOutputs() {
        wantedState = State.Nothing

        PeriodicIO.leftDemand = 0.0
        PeriodicIO.rightDemand = 0.0
    }

    private object PeriodicIO {
        // Inputs
        var leftVoltage: Double = 0.0
        var rightVoltage: Double = 0.0

        var leftCurrent: Double = 0.0
        var rightCurrent: Double = 0.0

        var leftRawSensorPosition: Double = 0.0
        var rightRawSensorPosition: Double = 0.0

        var leftRawSensorVelocity: Double = 0.0
        var rightRawSensorVelocity: Double = 0.0

        // Outputs
        var leftDemand: Double = 0.0
        var rightDemand: Double = 0.0

        var leftFeedforward: Double = 0.0
        var rightFeedforward: Double = 0.0
    }

    private enum class State {
        PathFollowing, OpenLoop, Nothing
    }
}