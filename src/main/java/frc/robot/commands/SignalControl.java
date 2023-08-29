// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShootingSubsystem;

public class SignalControl extends CommandBase {
  /** Creates a new SignalControl. */
  public SignalControl(ShootingSubsystem in) {
    // Use addRequirements() here to declare subsystem dependencies.
    shootingSubsystem = in;
    addRequirements(in);
  }

  public ShootingSubsystem shootingSubsystem;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.ctrlJoystick.getRawButtonPressed(4)) { // intake
      shootingSubsystem.serializer.set(0.15);
      Timer.delay(0.6);
      shootingSubsystem.serializer.set(0);
    }
    SmartDashboard.putBoolean("5", RobotContainer.ctrlJoystick.getRawButton(5));
    if (RobotContainer.ctrlJoystick.getRawButton(5)) {
      shootingSubsystem.intakeMotor.set(0.25);
    }
    else if (RobotContainer.ctrlJoystick.getRawButton(2)) {
      shootingSubsystem.serializer.set(-0.2);
      shootingSubsystem.intakeMotor.set(-0.2);
    } else {
      shootingSubsystem.serializer.set(0);
      shootingSubsystem.intakeMotor.set(0);
    }
    if (RobotContainer.ctrlJoystick.getRawButtonPressed(3)) { //shoot
      shootingSubsystem.intakeMotor.set(0.2);
      if(shootingSubsystem.distance>420) shootingSubsystem.shooter.set(0.55);
      else shootingSubsystem.shooter.set(0.5);
      Timer.delay(1);
      shootingSubsystem.serializer.set(0.15);
      Timer.delay(2);
      shootingSubsystem.shooter.set(0);
      shootingSubsystem.serializer.set(0);
      shootingSubsystem.intakeMotor.set(0);
    }
    if (RobotContainer.ctrlJoystick.getRawButton(1)) { //aim
      shootingSubsystem.aim();
    } 
    if (RobotContainer.ctrlJoystick.getRawButtonPressed(6)) {
      shootingSubsystem.shootLow();
    }
    // if (shootingSubsystem.angleEncoder.getSelectedSensorPosition() < Constants.angleZeroPos) {
    //   shootingSubsystem.anglePosDown = true;
    // } else {
    //   shootingSubsystem.anglePosDown = false;
    // }
    // shootingSubsystem.angleEncoder.set(RobotContainer.ctrlJoystick.getRawAxis(1))
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
