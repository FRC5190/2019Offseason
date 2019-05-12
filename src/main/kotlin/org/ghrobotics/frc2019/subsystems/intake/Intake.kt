package org.ghrobotics.frc2019.subsystems.intake

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.ghrobotics.frc2019.Constants

object Intake {

    val masterMotor = TalonSRX(Constants.kIntakeLeftId)
    val isHoldingHatch = false

}