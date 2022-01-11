// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class ControllerMap {
    private Joystick joystickleft;
    private Joystick joystickright;
    private Joystick joystickmanipulator;
    private double wristMoveSpeed;
    private boolean intakeOn;


    public ControllerMap(){
        joystickleft = new Joystick(0);
        joystickright = new Joystick(1);
        joystickmanipulator = new Joystick(2);
        checkAllControls();
    }

    public void periodic() {
        checkAllControls();
    }

    private void checkAllControls() {
        intakeOn = joystickright.getRawButton(2);
        wristMoveSpeed = joystickmanipulator.getRawAxis(1);
    }

    public double getWristSpeed(){
        return wristMoveSpeed;
    }
    public boolean isIntakeOn(){
        return intakeOn;
    }


}
