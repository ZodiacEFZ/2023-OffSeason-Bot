// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.spec.ECPublicKeySpec;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */
  public SwerveDrive(SwerveSubsystem ss) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSubsystem = ss;
    addRequirements(ss);
  }

  public static SwerveDrive instance;

  public static SwerveDrive getInstance() {
    if (instance == null) {
      instance = new SwerveDrive(SwerveSubsystem.getInstance());
    }
    return instance;
  }

  public final SwerveSubsystem swerveSubsystem;
  public boolean field_oriented = false, flag = false;
  public double targetAngle = 0, deadZone = 0.1, kv=1;
  public double[] angleGoal = new double[8], velocityGoal = new double[8];

  public double x_value, y_value, rot_value;
  public boolean aimmingState = false;
  public int aimmingOrien = 0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    field_oriented = false;
  }

  public void stop_all() {
    RobotContainer.LeftBackSwerveModule.setStill();
    RobotContainer.LeftFrontSwerveModule.setStill();
    RobotContainer.RightBackSwerveModule.setStill();
    RobotContainer.RightFrontSwerveModule.setStill();

    for (int i = 1; i <= 4; i++) {
      angleGoal[i] = 0;
      velocityGoal[i] = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("angle", swerveSubsystem.get_field_angle());
    x_value = RobotContainer.driveJoystick.getRawAxis(0);
    y_value = -RobotContainer.driveJoystick.getRawAxis(1);
    rot_value = -RobotContainer.driveJoystick.getRawAxis(4);

    SmartDashboard.putBoolean("aimming", aimmingState);

    double aimRange = 5;
    if (RobotContainer.ctrlJoystick.getRawButton(1)) {
      aimmingState = true;
    } else {
      aimmingState = false;
    }
    if(RobotContainer.driveJoystick.getRawButton(5)){
      kv=0.2;
    }
    else{
      kv=1;
    }
    SmartDashboard.putNumber("kv", kv);
    // if (aimmingState) {
    //   if (!ShootingSubsystem.getInstance().spotted) {
    //     RobotContainer.ctrlRumble();
    //     stop_all();
    //     aimmingState = false;
    //   }
    // }
    // System.out.print(rot_value);
    if (!aimmingState) {
      if (RobotContainer.driveJoystick.getRawButtonPressed(1)) {
        if (field_oriented) {
          // Current state is true so turn off
          field_oriented = false;
          RobotContainer.driveRumble();
        } else {
          // Current state is false so turn on
          field_oriented = true;
          RobotContainer.driveRumble();
        }
      }
      // field_oriented=false;
      if (!flag) {
        targetAngle = swerveSubsystem.get_field_angle();
      }
      SmartDashboard.putBoolean("field_oriented", field_oriented);
      SmartDashboard.putNumber("targetangle", targetAngle);
      if (Math.abs(x_value) < deadZone)
        x_value = 0;
      if (Math.abs(y_value) < deadZone)
        y_value = 0;
      if (Math.abs(rot_value) < deadZone)
        rot_value = 0;
      if (Math.abs(x_value) < deadZone && Math.abs(y_value) < deadZone && Math.abs(rot_value) < deadZone) {
        stop_all();
        flag = false;
      } else {
        if (Math.abs(rot_value) < deadZone)
          flag = true;
        else
          flag = false;
        if (field_oriented) {
          swerveSubsystem.field_oriented(x_value, y_value, rot_value,
              Math.toRadians(swerveSubsystem.get_field_angle()));
        } else {
          if (flag) {
            double error = targetAngle - swerveSubsystem.get_field_angle();
            // error = -error; //seems wrong
            if (error > 180)
              error -= 360;
            else if (error < -180)
              error += 360;
            // error=0;
            rot_value = error * 0.02;
            if(rot_value>0.5) rot_value=0.5;
            if(rot_value<-0.5)  rot_value=-0.5;
            SmartDashboard.putNumber("error", error);
          }
          swerveSubsystem.car_oriented(x_value, y_value, rot_value);

          SmartDashboard.putNumber("x_axis", x_value);
          SmartDashboard.putNumber("y_axis", y_value);
          SmartDashboard.putNumber("z_axis", rot_value);

        }

        angleGoal = swerveSubsystem.get_theta();
        velocityGoal = swerveSubsystem.get_velocity();

        for (int i = 1; i <= 4; i++) {
          angleGoal[i] = (Math.toDegrees(angleGoal[i])) % 360;
          velocityGoal[i] = 18000 * velocityGoal[i] + 2000;
        }

        SmartDashboard.putNumberArray("rawAngleGoal", angleGoal);

        RobotContainer.LeftFrontSwerveModule.setStatus(angleGoal[1], velocityGoal[1]*kv);
        RobotContainer.RightFrontSwerveModule.setStatus(angleGoal[2], velocityGoal[2]*kv);
        RobotContainer.RightBackSwerveModule.setStatus(angleGoal[3], velocityGoal[3]*kv);
        RobotContainer.LeftBackSwerveModule.setStatus(angleGoal[4], velocityGoal[4]*kv);
      }
    } else {
      double crx=ShootingSubsystem.getInstance().targetX,error=-crx/80;
      SmartDashboard.putNumber("CRX", crx);
      if(error>0.2) error=0.2;
      if(error<-0.2)  error=-0.2; 
      SmartDashboard.putNumber("error", error);
      if (Math.abs(error)>0.06) {
        swerveSubsystem.car_oriented(0, 0, error);
      } 
       else {
        RobotContainer.ctrlRumble();
        stop_all();
        aimmingState = false;
      }
    }
    if (aimmingState) {
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
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
