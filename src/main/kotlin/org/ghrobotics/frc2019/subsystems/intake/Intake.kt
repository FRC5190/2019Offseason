package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Solenoid
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.inch

object Intake : FalconSubsystem(), Loggable {

    val masterMotor = TalonSRX(Constants.kIntakeLeftId)

    private val periodicIO = PeriodicIO()

    private val extensionSolenoid = DoubleSolenoid(
        Constants.kPCMId,
        Constants.kIntakeExtensionSolenoidForwardId,
        Constants.kIntakeExtensionSolenoidReverseId
    )
    private val launcherSolenoid = Solenoid(Constants.kPCMId, Constants.kIntakeLauncherSolenoidId)

    private val leftBallSensor = AnalogInput(Constants.kLeftBallSensorId)
    private val rightBallSensor = AnalogInput(Constants.kRightBallSensorId)

    val isHoldingHatch get() = periodicIO.hasHatchPanel
    val isHoldingCargo get() = periodicIO.hasCargo
    val extensionSolenoidState get() = periodicIO.extensionState
    val launcherSolenoidState get() = periodicIO.launcherState

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

        periodicIO.voltage = masterMotor.motorOutputVoltage
        periodicIO.current = masterMotor.outputCurrent

        periodicIO.hasHatchPanel = false
        periodicIO.hasCargo = leftBallSensor.averageVoltage > 1.7 || rightBallSensor.averageVoltage > 1.2

        masterMotor.set(ControlMode.PercentOutput, periodicIO.demand)
    }

    fun setExtensionSolenoid(extended: Boolean) {
        periodicIO.extensionState = extended
        extensionSolenoid.set(
            if (extended) {
                DoubleSolenoid.Value.kForward
            } else {
                DoubleSolenoid.Value.kReverse
            }
        )
    }

    fun setLauncherSolenoid(extended: Boolean) {
        periodicIO.launcherState = extended
        launcherSolenoid.set(extended)
    }

    fun setOpenLoop(percent: Double) {
        periodicIO.demand = percent
    }

    override fun zeroOutputs() {
        periodicIO.demand = 0.0
    }

    private class PeriodicIO : Loggable {
        // Inputs
        @Log.VoltageView(name = "Voltage")
        var voltage: Double = 0.0

        @Log(name = "Current")
        var current: Double = 0.0

        var hasHatchPanel: Boolean = false

        @Log(name = "Has Cargo")
        var hasCargo: Boolean = false

        var extensionState = false
        var launcherState = false

        // Outputs
        var demand: Double = 0.0
    }
}