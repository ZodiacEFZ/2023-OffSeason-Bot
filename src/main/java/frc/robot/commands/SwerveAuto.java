// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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
  public double[] angleGoal = new double[8], velocityGoal = new double[8];
  public boolean aimmingState, end = false, moving = true;
  public double targetangle;
  public double t,start;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetangle = swerveSubsystem.get_field_angle();
    aimmingState = true;
    shootingSubsystem.shootLow();
    start=Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    t=Timer.getFPGATimestamp();
    SmartDashboard.putNumber("time", t);
    if (moving) {
      double error = targetangle - swerveSubsystem.get_field_angle();
      if (error > 180)
        error -= 360;
      else if (error < -180)
        error += 360;
      // error=0;
      swerveSubsystem.car_oriented(0, -0.2, error * 0.005);
      angleGoal = swerveSubsystem.get_theta();
      velocityGoal = swerveSubsystem.get_velocity();
  
      for (int i = 1; i <= 4; i++) {
        angleGoal[i] = (Math.toDegrees(angleGoal[i])) % 360;
        velocityGoal[i] = 18000 * velocityGoal[i] + 2000;
      }
      
      RobotContainer.LeftFrontSwerveModule.setStatus(angleGoal[1], velocityGoal[1]);
      RobotContainer.RightFrontSwerveModule.setStatus(angleGoal[2], velocityGoal[2]);
      RobotContainer.RightBackSwerveModule.setStatus(angleGoal[3], velocityGoal[3]);
      RobotContainer.LeftBackSwerveModule.setStatus(angleGoal[4], velocityGoal[4]);
      if(/*shootingSubsystem.spotted&&shootingSubsystem.distance>300&&*/(t-start)>=2){
        moving=false;
        RobotContainer.LeftFrontSwerveModule.setStatus(0, 0);
        RobotContainer.RightFrontSwerveModule.setStatus(0, 0);
        RobotContainer.RightBackSwerveModule.setStatus(0, 0);
        RobotContainer.LeftBackSwerveModule.setStatus(0, 0);
      }
     } //else {
    //  /* if (aimmingState) {
    //     double crx = ShootingSubsystem.getInstance().targetX, error = -crx / 100;
    //     if (error > 0.2)
    //       error = 0.2;
    //     if (error < -0.2)
    //       error = -0.2;
    //     if (Math.abs(error) > 0.1) {
    //       swerveSubsystem.car_oriented(0, 0, error);
    //     } else {
    //       RobotContainer.LeftFrontSwerveModule.setStatus(0, 0);
    //       RobotContainer.RightFrontSwerveModule.setStatus(0, 0);
    //       RobotContainer.RightBackSwerveModule.setStatus(0, 0);
    //       RobotContainer.LeftBackSwerveModule.setStatus(0, 0);
    //       aimmingState = false;
    //     }
    //   } else {
    //     shootingSubsystem.shooter.set(0.6);
    //     Timer.delay(1);
    //     shootingSubsystem.serializer.set(0.2);
    //     Timer.delay(2);
    //     shootingSubsystem.shooter.set(0);
    //     shootingSubsystem.serializer.set(0);
    //     end = true;
    //   }*/
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
