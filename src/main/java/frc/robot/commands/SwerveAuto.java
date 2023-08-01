// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveAuto extends CommandBase {
  /** Creates a new SwerveAuto. */
  public SwerveAuto(SwerveSubsystem ss, ShootingSubsystem in) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSubsystem = ss;
    shootingSubsystem = in;
    addRequirements(ss, in);
  }

  private final SwerveSubsystem swerveSubsystem;
  private final ShootingSubsystem shootingSubsystem;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.car_oriented(0, -0.2, 0);
    Timer.delay(1);
    swerveSubsystem.car_oriented(0, 0, 0);
    if (shootingSubsystem.aim()) {
      shootingSubsystem.shoot();
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
