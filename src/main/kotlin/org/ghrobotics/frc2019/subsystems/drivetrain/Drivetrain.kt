package org.ghrobotics.frc2019.subsystems.drivetrain

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.sensors.PigeonIMU
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Config
import io.github.oblarg.oblog.annotations.Log
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.intake.Intake
import org.ghrobotics.lib.commands.ConditionCommand
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem
import kotlin.properties.Delegates

object Drivetrain : TankDriveSubsystem(), EmergencyHandleable, Loggable {

    override val leftMotor = configureDriveGearbox(Constants.kDriveLeftMasterId, Constants.kDriveLeftSlaveId, false)
    override val rightMotor = configureDriveGearbox(Constants.kDriveRightMasterId, Constants.kDriveRightSlaveId, true)

    private val periodicIO = PeriodicIO()

    @Log.ToString(name = "Current State", rowIndex = 0, columnIndex = 5)
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    override val differentialDrive = Constants.kDriveModel
    override val trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

    private val gyro = PigeonIMU(Intake.masterMotor)
    private val shifter = Solenoid(Constants.kPCMId, Constants.kDriveSolenoidId)

    override val localization = TankEncoderLocalization(
        { angle },
        { lPosition.value },
        { rPosition.value }
    )

    val lPosition: Length
        get() = Constants.kDriveNativeUnitModel.fromNativeUnitPosition(periodicIO.leftRawSensorPosition.nativeUnits)

    val rPosition: Length
        get() = Constants.kDriveNativeUnitModel.fromNativeUnitPosition(periodicIO.rightRawSensorPosition.nativeUnits)

    val angle: Rotation2d
        get() = periodicIO.gyroAngle.degree

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

    fun notWithinRegion(region: Rectangle2d) =
        ConditionCommand { !region.contains(robotPosition.translation) }

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
        periodicIO.leftVoltage = leftMotor.voltageOutput
        periodicIO.rightVoltage = rightMotor.voltageOutput

        periodicIO.leftCurrent = leftMotor.talonSRX.outputCurrent
        periodicIO.rightCurrent = rightMotor.talonSRX.outputCurrent

        periodicIO.leftRawSensorPosition = leftMotor.encoder.rawPosition
        periodicIO.rightRawSensorPosition = rightMotor.encoder.rawPosition

        periodicIO.leftRawSensorVelocity = leftMotor.encoder.rawVelocity
        periodicIO.rightRawSensorVelocity = rightMotor.encoder.rawVelocity

        periodicIO.gyroAngle = gyro.fusedHeading

        when (wantedState) {
            State.Nothing -> {
                leftMotor.setNeutral()
                rightMotor.setNeutral()
            }
            State.PathFollowing -> {
                leftMotor.setVelocity(periodicIO.leftDemand, periodicIO.leftFeedforward)
                rightMotor.setVelocity(periodicIO.rightDemand, periodicIO.rightFeedforward)
            }
            State.OpenLoop -> {
                leftMotor.setDutyCycle(periodicIO.leftDemand)
                rightMotor.setDutyCycle(periodicIO.rightDemand)
            }
        }
        if (currentState != wantedState) currentState = wantedState
    }

    override fun tankDrive(leftPercent: Double, rightPercent: Double) = setOpenLoop(leftPercent, rightPercent)

    fun setOpenLoop(left: Double, right: Double) {
        wantedState = State.OpenLoop

        periodicIO.leftDemand = left
        periodicIO.rightDemand = right

        periodicIO.leftFeedforward = 0.0
        periodicIO.rightFeedforward = 0.0
    }


    override fun setOutput(wheelVelocities: DifferentialDrive.WheelState, wheelVoltages: DifferentialDrive.WheelState) {
        wantedState = State.PathFollowing

        periodicIO.leftDemand = wheelVelocities.left * differentialDrive.wheelRadius
        periodicIO.rightDemand = wheelVelocities.right * differentialDrive.wheelRadius

        periodicIO.leftFeedforward = wheelVoltages.left
        periodicIO.rightFeedforward = wheelVoltages.right
    }

    override fun zeroOutputs() {
        wantedState = State.Nothing

        periodicIO.leftDemand = 0.0
        periodicIO.rightDemand = 0.0
    }

    private class PeriodicIO : Loggable {

        override fun configureLayoutType() = BuiltInLayouts.kGrid
        override fun skipLayout() = true

        // Inputs
        @Log.VoltageView(name = "Left Voltage", width = 2, height = 1, rowIndex = 0, columnIndex = 0)
        var leftVoltage: Double = 0.0

        @Log.VoltageView(name = "Right Voltage", width = 2, height = 1, rowIndex = 0, columnIndex = 2)
        var rightVoltage: Double = 0.0


        @Log(name = "Left Current", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
        var leftCurrent: Double = 0.0

        @Log(name = "Right Current", width = 2, height = 1, rowIndex = 1, columnIndex = 2)
        var rightCurrent: Double = 0.0


        @Log(name = "Left Sensor Pos", width = 2, height = 1, rowIndex = 2, columnIndex = 0)
        var leftRawSensorPosition: Double = 0.0

        @Log(name = "Right Sensor Pos", width = 2, height = 1, rowIndex = 2, columnIndex = 2)
        var rightRawSensorPosition: Double = 0.0

        @Log(name = "Gyro Angle", rowIndex = 3, columnIndex = 0)
        var gyroAngle = 0.0

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