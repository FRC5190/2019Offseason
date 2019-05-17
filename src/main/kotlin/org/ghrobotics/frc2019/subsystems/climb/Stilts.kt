package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.RemoteSensorSource
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.nativeunits.nativeUnits
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable

object Stilts : FalconSubsystem(), EmergencyHandleable, Loggable {

    private val frontMasterMotor = FalconSRX(
        Constants.kClimbFrontWinchMasterId,
        Constants.kClimbFrontWinchNativeUnitModel
    )
    private val backMasterMotor = FalconSRX(
        Constants.kClimbBackWinchMasterId,
        Constants.kClimbBackWinchNativeUnitModel
    )

    @Log(name = "PeriodicIO")
    private val periodicIO = PeriodicIO()

    private val backHallEffectSensor = DigitalInput(Constants.kClimberHallEffectSensor)

    @Log.ToString(name = "Current State")
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    val frontHeight: Length
        get() = Constants.kClimbFrontWinchNativeUnitModel.fromNativeUnitPosition(periodicIO.frontRawSensorPosition.nativeUnits)

    val backHeight: Length
        get() = Constants.kClimbBackWinchNativeUnitModel.fromNativeUnitPosition(periodicIO.backRawSensorPosition.nativeUnits)

    val frontLimitSwitchEngaged: Boolean
        get() = periodicIO.frontLimitSwitch

    val backLimitSwitchEngaged: Boolean
        get() = periodicIO.backLimitSwitch

    init {
        listOf(frontMasterMotor, backMasterMotor).forEach { master ->
            master.outputInverted = true
            master.encoder.encoderPhase = true

            master.brakeMode = true

            master.voltageCompSaturation = 12.0

            master.configCurrentLimit(
                true, FalconSRX.CurrentLimitConfig(
                    0.amp,
                    0.second,
                    Constants.kClimbWinchCurrentLimit
                )
            )

            master.talonSRX.configRemoteFeedbackFilter(
                Constants.kPigeonIMUId,
                RemoteSensorSource.GadgeteerPigeon_Roll,
                0
            )

            master.talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
            master.talonSRX.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 10)

            master.talonSRX.selectProfileSlot(0, 0)
            master.talonSRX.selectProfileSlot(2, 1)

            master.useMotionProfileForPosition = true

            // Motion Magic
            master.talonSRX.config_kP(0, Constants.kClimbWinchPositionKp, 10)

            // Position PID
            master.talonSRX.config_kP(1, Constants.kClimbWinchPositionKp, 10)

            // Leveling Aux
            master.talonSRX.config_kP(2, Constants.kClimbWinchLevelingKp, 10)
            master.talonSRX.config_kD(2, Constants.kClimbWinchLevelingKd, 10)
        }

        frontMasterMotor.motionProfileAcceleration = 0.8.feet.acceleration.value
        frontMasterMotor.motionProfileCruiseVelocity = 1.65.feet.velocity.value

        backMasterMotor.motionProfileAcceleration = 0.65.feet.acceleration.value
        backMasterMotor.motionProfileCruiseVelocity = 1.2.feet.velocity.value

        frontMasterMotor.talonSRX.configAuxPIDPolarity(false)
        backMasterMotor.talonSRX.configAuxPIDPolarity(true)

        defaultCommand = TeleopStiltsCommand()
    }

    override fun activateEmergency() {
        listOf(frontMasterMotor, backMasterMotor).forEach { master ->
            // Motion Magic
            master.talonSRX.config_kP(0, 0.0, 10)

            // Position PID
            master.talonSRX.config_kP(1, 0.0, 10)

            // Leveling Aux
            master.talonSRX.config_kP(2, 0.0, 10)
            master.talonSRX.config_kD(2, 0.0, 10)
        }
        zeroOutputs()
    }

    override fun recoverFromEmergency() {
        listOf(frontMasterMotor, backMasterMotor).forEach { master ->
            // Motion Magic
            master.talonSRX.config_kP(0, Constants.kClimbWinchPositionKp, 10)

            // Position PID
            master.talonSRX.config_kP(1, Constants.kClimbWinchPositionKp, 10)

            // Leveling Aux
            master.talonSRX.config_kP(2, Constants.kClimbWinchLevelingKp, 10)
            master.talonSRX.config_kD(2, Constants.kClimbWinchLevelingKd, 10)
        }
    }

    override fun periodic() {
        periodicIO.frontVoltage = frontMasterMotor.voltageOutput
        periodicIO.backVoltage = backMasterMotor.voltageOutput

        periodicIO.frontCurrent = frontMasterMotor.talonSRX.outputCurrent
        periodicIO.backCurrent = backMasterMotor.talonSRX.outputCurrent

        periodicIO.frontRawSensorPosition = frontMasterMotor.encoder.rawPosition
        periodicIO.backRawSensorPosition = backMasterMotor.encoder.rawPosition

        periodicIO.frontRawSensorVelocity = frontMasterMotor.encoder.rawVelocity
        periodicIO.backRawSensorVelocity = backMasterMotor.encoder.rawVelocity

        periodicIO.frontLimitSwitch = frontMasterMotor.talonSRX.sensorCollection.isRevLimitSwitchClosed
        periodicIO.backLimitSwitch = !backHallEffectSensor.get()

        if (periodicIO.backLimitSwitch) {
            periodicIO.backDemand = periodicIO.backDemand.coerceAtLeast(0.0)
        }

        when (wantedState) {
            State.Nothing -> {
                frontMasterMotor.setNeutral()
                backMasterMotor.setNeutral()
            }
            State.AuxPIDMotionMagic -> {
                frontMasterMotor.talonSRX.set(ControlMode.MotionMagic, periodicIO.frontDemand, DemandType.AuxPID, 0.0)
                backMasterMotor.talonSRX.set(ControlMode.MotionMagic, periodicIO.backDemand, DemandType.AuxPID, 0.0)
            }
            State.MotionMagic -> {
                frontMasterMotor.setPosition(periodicIO.frontDemand)
                backMasterMotor.setPosition(periodicIO.backDemand)
            }
            State.OpenLoop -> {
                frontMasterMotor.setDutyCycle(periodicIO.frontDemand)
                backMasterMotor.setDutyCycle(periodicIO.backDemand)
            }
        }
        if (currentState != wantedState) currentState = wantedState
    }

    fun setAutoClimbHeight(frontHeight: Length, backHeight: Length) {
        if (currentState != State.AuxPIDMotionMagic) {
            frontMasterMotor.talonSRX.selectProfileSlot(0, 0)
            backMasterMotor.talonSRX.selectProfileSlot(0, 0)
        }
        wantedState = State.AuxPIDMotionMagic

        periodicIO.frontDemand = frontHeight.value
        periodicIO.backDemand = backHeight.value
    }

    fun setMotionMagic(frontHeight: Length, backHeight: Length) {
        if (currentState != State.MotionMagic) {
            frontMasterMotor.talonSRX.selectProfileSlot(1, 0)
            backMasterMotor.talonSRX.selectProfileSlot(1, 0)
        }
        wantedState = State.MotionMagic

        periodicIO.frontDemand = frontHeight.value
        periodicIO.backDemand = backHeight.value
    }

    fun setOpenLoop(frontPercent: Double, backPercent: Double) {
        wantedState = State.OpenLoop

        periodicIO.frontDemand = frontPercent
        periodicIO.backDemand = backPercent
    }

    override fun zeroOutputs() {
        wantedState = State.Nothing

        periodicIO.frontDemand = 0.0
        periodicIO.backDemand = 0.0
    }

    private class PeriodicIO : Loggable {

        override fun configureLayoutType() = BuiltInLayouts.kGrid

        // Inputs
        @Log.VoltageView(name = "Front Voltage", width = 2, height = 1, rowIndex = 0, columnIndex = 0)
        var frontVoltage: Double = 0.0

        @Log.VoltageView(name = "Back Voltage", width = 2, height = 1, rowIndex = 0, columnIndex = 2)
        var backVoltage: Double = 0.0

        @Log(name = "Front Current", width = 2, height = 1, rowIndex = 1, columnIndex = 0)
        var frontCurrent: Double = 0.0

        @Log(name = "Back Current", width = 2, height = 1, rowIndex = 1, columnIndex = 2)
        var backCurrent: Double = 0.0

        @Log(name = "Front Raw Sensor Pos", width = 2, height = 1, rowIndex = 2, columnIndex = 0)
        var frontRawSensorPosition: Double = 0.0

        @Log(name = "Back Raw Sensor Pos", width = 2, height = 1, rowIndex = 2, columnIndex = 2)
        var backRawSensorPosition: Double = 0.0

        var frontRawSensorVelocity: Double = 0.0
        var backRawSensorVelocity: Double = 0.0

        @Log(name = "Front Limit Engaged", width = 2, height = 1, rowIndex = 3, columnIndex = 0)
        var frontLimitSwitch: Boolean = false

        @Log(name = "Back Limit Engaged", width = 2, height = 1, rowIndex = 3, columnIndex = 2)
        var backLimitSwitch: Boolean = false

        // Outputs
        var frontDemand: Double = 0.0
        var backDemand: Double = 0.0
    }

    private enum class State {
        AuxPIDMotionMagic, MotionMagic, OpenLoop, Nothing
    }
}
