// https://prod.liveshare.vsengsaas.visualstudio.com/join?BAD79CED7DC60533A44EF895A3DF03A411C6



// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/** Add your docs here. */
public class Arm {
    private TalonSRX wrist;
    private TalonSRX pivot;
    private TalonSRX lift;


    private Arm() {
      wrist = new TalonSRX(RobotMap.ARM_WRIST);
      pivot = new TalonSRX(RobotMap.ARM_PIVOT);
      lift = new TalonSRX(RobotMap.ARM_LIFT);
    
     }


     public void wristMoveSpeed(){
         wrist.set(ControlMode.PercentOutput, 0.05);
     }
}
