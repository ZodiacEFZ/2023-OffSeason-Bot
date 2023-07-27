// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {
    spinLeft.configFactoryDefault();
    spinLeft.setInverted(false);

    spinRight.configFactoryDefault();
    spinRight.follow(spinLeft);
    spinRight.setInverted(true);
  }

  private WPI_TalonFX spinLeft = new WPI_TalonFX(Constants.spinLeftPort);
  private WPI_TalonFX spinRight = new WPI_TalonFX(Constants.spinRightPort);

  private WPI_TalonFX arm = new WPI_TalonFX(Constants.armPort);

  private int upPos = 0, downPos = 0;
  public boolean downStatus = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.ctrlJoystick.getRawButtonPressed(1)) {
      
    }
  }

  public void intake() {
    spinLeft.set(0.5);
  }

  public void stop() {
    spinLeft.set(0);
  }

  public void up() {
    arm.set(ControlMode.Position, upPos);
    downStatus = false;
  }

  public void down() {
    arm.set(ControlMode.Position, downPos);
    downStatus = true;
  }
}
