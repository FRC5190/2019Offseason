package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.inch

object Intake : FalconSubsystem() {

    val masterMotor = TalonSRX(Constants.kIntakeLeftId)

    private val extensionSolenoid = DoubleSolenoid(
        Constants.kPCMId,
        Constants.kIntakeExtensionSolenoidForwardId,
        Constants.kIntakeExtensionSolenoidReverseId
    )
    private val launcherSolenoid = Solenoid(Constants.kPCMId, Constants.kIntakeLauncherSolenoidId)

    private val leftBallSensor = AnalogInput(Constants.kLeftBallSensorId)
    private val rightBallSensor = AnalogInput(Constants.kRightBallSensorId)

    val isHoldingHatch get() = PeriodicIO.hasHatchPanel
    val isHoldingCargo get() = PeriodicIO.hasCargo
    val extensionSolenoidState get() = PeriodicIO.extensionState
    val launcherSolenoidState get() = PeriodicIO.launcherState

    var badIntakeOffset = (-.15).inch

    var robotPositionWithIntakeOffset = Pose2d()
        private set

    init {
        val slave1Motor = TalonSRX(Constants.kIntakeRightId)
        slave1Motor.follow(masterMotor)

        masterMotor.inverted = true
        slave1Motor.inverted = false

        fun configMotor(motor: TalonSRX) {
            motor.configVoltageCompSaturation(12.0)
            motor.enableVoltageCompensation(true)

            motor.configContinuousCurrentLimit(25)
            motor.enableCurrentLimit(true)

            motor.setNeutralMode(NeutralMode.Brake)
        }

        configMotor(masterMotor)
        configMotor(slave1Motor)

        setExtensionSolenoid(false)
        setLauncherSolenoid(false)
    }

    override fun periodic() {
        robotPositionWithIntakeOffset = Drivetrain.robotPosition + Pose2d(Length.kZero, -badIntakeOffset)

        PeriodicIO.voltage = masterMotor.motorOutputVoltage
        PeriodicIO.current = masterMotor.outputCurrent

        PeriodicIO.hasHatchPanel = false
        PeriodicIO.hasCargo = leftBallSensor.averageVoltage > 1.7 || rightBallSensor.averageVoltage > 1.2

        masterMotor.set(ControlMode.PercentOutput, PeriodicIO.demand)
    }

    fun setExtensionSolenoid(extended: Boolean) {
        PeriodicIO.extensionState = extended
        extensionSolenoid.set(
            if (extended) {
                DoubleSolenoid.Value.kForward
            } else {
                DoubleSolenoid.Value.kReverse
            }
        )
    }

    fun setLauncherSolenoid(extended: Boolean) {
        PeriodicIO.launcherState = extended
        launcherSolenoid.set(extended)
    }

    fun setOpenLoop(percent: Double) {
        PeriodicIO.demand = percent
    }

    override fun zeroOutputs() {
        PeriodicIO.demand = 0.0
    }

    private object PeriodicIO {
        // Inputs
        var voltage: Double = 0.0
        var current: Double = 0.0

        var hasHatchPanel: Boolean = false
        var hasCargo: Boolean = false

        var extensionState = false
        var launcherState = false

        // Outputs
        var demand: Double = 0.0
    }
}