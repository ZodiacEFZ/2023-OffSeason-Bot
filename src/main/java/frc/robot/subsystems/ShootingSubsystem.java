// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SwerveDrive;
import frc.robot.lib.LinearInterpolationTable;

import java.awt.geom.Point2D;

public class ShootingSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  public ShootingSubsystem() {
    angleEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    angleControlTalonFX.setInverted(true);
    angleControlTalonFX.setNeutralMode(NeutralMode.Brake);

    shooter.configFactoryDefault();
    shooter.config_kP(0, 0.3);
    shooter.config_kI(0, 0);
    shooter.config_kD(0, 0);
    shooter.config_kF(0, 0);
    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter.configVoltageCompSaturation(10);
    shooter.enableVoltageCompensation(true);
    shooter.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
    shooter.configVelocityMeasurementWindow(1);
    shooter.setInverted(true);

    intakeEncoder.setSensorPhase(true);
    intakeEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    intakeArm.configFactoryDefault();
    intakeArm.config_kP(0, 0.1);
    intakeArm.config_kP(0, 0);
    intakeArm.config_kP(0, 0);
    intakeArm.setNeutralMode(NeutralMode.Brake);

    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);
  }

  public WPI_TalonFX shooter = new WPI_TalonFX(Constants.spinMotorPort);
  public WPI_TalonFX serializer = new WPI_TalonFX(Constants.serializerPort);
  public WPI_TalonFX angleControlTalonFX = new WPI_TalonFX(Constants.angleControlPort);
  public WPI_TalonSRX angleEncoder = new WPI_TalonSRX(Constants.angleEncoderPort);
  public WPI_TalonFX intakeArm = new WPI_TalonFX(Constants.intakeArmPort);
  public WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.intakeMotorPort);

  private WPI_TalonSRX intakeEncoder = new WPI_TalonSRX(Constants.intakeEncoderPort);

  public int upPos = 0;
  public int downPos = 0;
  public boolean downStatus = false;

  public boolean anglePosDown = true;
  private int angleZeroPos = 0;
  private double anglePos = 0;

  public double targetHeight = 264, limelightHeight = 52; // all in centimetres
  public double limelightAngle = 51.6 * 3.14159 / 180, targetAngle = 0; // all in radians

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");

  public double distance = 0;

  public double targetX = 0;
  public double targetY = 0;
  public double targetArea = 0;
  public boolean spotted = false;

  private static final Point2D[] kAngles = new Point2D.Double[] {
      // distance(cm), angle(pos)
      new Point2D.Double(1, 2),
  };

  private static final LinearInterpolationTable kAngleTable = new LinearInterpolationTable(kAngles);

  private static final Point2D[] kRPMs = new Point2D.Double[] {
      // distance(cm), RPM
      new Point2D.Double(),
  };

  private static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPMs);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetX = tx.getDouble(0.0);
    targetY = ty.getDouble(0.0);
    targetArea = ta.getDouble(0.0);
    spotted = (tv.getDouble(0) == 1 ? true : false);
    SmartDashboard.putNumber("Ll X", targetX);
    SmartDashboard.putNumber("Ll Y", targetY);
    SmartDashboard.putNumber("Ll a", targetArea);
    SmartDashboard.putBoolean("Ll status", spotted);

    targetAngle = targetY * 3.14159 / 180;
    SmartDashboard.putNumber("targetAngle", targetAngle);

    anglePos = angleEncoder.getSelectedSensorPosition();
    SmartDashboard.putNumber("anglePos", anglePos);

    distance = (targetHeight - limelightHeight) / Math.tan(limelightAngle + targetAngle);
    SmartDashboard.putNumber("distance", distance);

    SmartDashboard.putNumber("shooterVelocity", shooter.getSelectedSensorVelocity());
  }

  public void up() {
    intakeArm.set(ControlMode.Position, upPos);
    intakeMotor.set(0);
  }

  public void down() {
    intakeArm.set(ControlMode.Position, downPos);
    intakeMotor.set(0.2);
  }

  public boolean aim() {
    double aimRange = 1;
    if (!spotted) {
      RobotContainer.ctrlRumble();
      return false;
    }
    while ((targetX > aimRange || targetX < aimRange * -1) && spotted) {
      if (targetX > aimRange) {
        SwerveDrive.getInstance().swerveSubsystem.car_oriented(0, 0, 0.25 * (targetX / 24));
      } else if (targetX < aimRange * -1) {
        SwerveDrive.getInstance().swerveSubsystem.car_oriented(0, 0, -0.25 * (targetX / 24));
      } else {
        RobotContainer.ctrlRumble();
        return true;
      }
    }
    return false;
  }

  public void shoot() {
    double shootRPM = kRPMTable.getOutput(distance);
    double shootAngle = kAngleTable.getOutput(distance);
    angleControlTalonFX.set(ControlMode.Position, shootAngle);
    shooter.set(ControlMode.Velocity, shootRPM);
    if (shooter.getSelectedSensorVelocity() >= shootRPM - 100) {
      serializer.set(0.2);
    }
    shooter.set(0);
  }
}

/*
 * List of shooting data:
 * Pos Dis
 * 
 * 
 * f(d) =
 */