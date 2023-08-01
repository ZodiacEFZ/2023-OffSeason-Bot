// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

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
    if (RobotContainer.ctrlJoystick.getRawButtonPressed(4)) { // TODO: check Falcon
      shootingSubsystem.down();
    } else if (RobotContainer.ctrlJoystick.getRawButtonPressed(4)) {
      shootingSubsystem.up();
    }
    // if (RobotContainer.ctrlJoystick.getRawButton(3)) {
    //   intake.shooter.set(0.4);
    // } else {
    //   intake.shooter.set(0);
    // }
    if (RobotContainer.ctrlJoystick.getRawButtonPressed(3)) {
      shootingSubsystem.shoot();
    } 
    if (RobotContainer.ctrlJoystick.getRawButton(2)) {
      shootingSubsystem.shooter.set(-0.1);
      shootingSubsystem.serializer.set(-0.4);
      shootingSubsystem.down();
      shootingSubsystem.intakeMotor.set(-0.2);
    } else {
      shootingSubsystem.shooter.set(0);
    }
    // if (RobotContainer.ctrlJoystick.getRawAxis(1) > 0.05) { // TODO: add man-control support
    //   intake.angleControlTalonFX.set(0.05);
    // }
    // if (RobotContainer.ctrlJoystick.getRawAxis(1) < -0.05 && !intake.anglePosDown) {
    //   intake.angleControlTalonFX.set(-0.05);
    // }
    if (shootingSubsystem.angleEncoder.getSelectedSensorPosition() < Constants.angleZeroPos) {
      shootingSubsystem.anglePosDown = true;
    } else {
      shootingSubsystem.anglePosDown = false;
    }

    if (RobotContainer.ctrlJoystick.getRawButtonPressed(1)) {
      shootingSubsystem.aim();
    }

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
