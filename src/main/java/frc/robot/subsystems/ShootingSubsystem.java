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
    //angleEncoder.setInverted(true);
    angleEncoder.config_kP(0, 0.15);
    angleEncoder.config_kI(0, 0.0001);
    angleEncoder.config_kD(0, 0);
    angleEncoder.config_kF(0, 0);
    angleEncoder.setNeutralMode(NeutralMode.Brake);
    
    shooter.configFactoryDefault();
    shooter.config_kP(0, 0.1);
    shooter.config_kI(0, 0.00025);
    shooter.config_kD(0, 0.01);
    shooter.config_kF(0, 0);
    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter.configVoltageCompSaturation(10);
    shooter.enableVoltageCompensation(true);
    shooter.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
    shooter.configVelocityMeasurementWindow(1);
    shooter.setInverted(true);

    intakeMotor.configFactoryDefault();
    intakeMotor.setInverted(false);

    serializer.configVoltageCompSaturation(10);
    serializer.enableVoltageCompensation(true);
    serializer.setNeutralMode(NeutralMode.Brake);
  }

  public static ShootingSubsystem instance;
  public static ShootingSubsystem getInstance() {
    if (instance == null) {
      instance = new ShootingSubsystem();
    }
    return instance;
  }

  public WPI_TalonFX shooter = new WPI_TalonFX(Constants.spinMotorPort);
  public WPI_TalonFX serializer = new WPI_TalonFX(Constants.serializerPort);
  public WPI_TalonFX angleControlTalonFX = new WPI_TalonFX(Constants.angleControlPort);
  public WPI_TalonSRX angleEncoder = new WPI_TalonSRX(Constants.angleEncoderPort);
 // public WPI_TalonFX intakeArm = new WPI_TalonFX(Constants.intakeArmPort);
  public WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.intakeMotorPort);

  //private WPI_TalonSRX intakeEncoder = new WPI_TalonSRX(Constants.intakeEncoderPort);

  public int upPos = 0;
  public int downPos = 0;
  public boolean downStatus = false;

  public boolean anglePosDown = true;
  private int angleZeroPos = 1200;
  private double anglePos = 0;

  public double targetHeight = 264, limelightHeight = 52; // all in centimetres
  public double limelightAngle = 45 * 3.14159 / 180, targetAngle = 0; // all in radians

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
      new Point2D.Double(350, -1700),
      new Point2D.Double(280, -460),
      new Point2D.Double(220, 30),
      new Point2D.Double(200, 424),
      new Point2D.Double(175, 875),
  };

  private static final LinearInterpolationTable kAngleTable = new LinearInterpolationTable(kAngles);

  private static final Point2D[] kRPMs = new Point2D.Double[] {
      // distance(cm), RPM
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

    //angleEncoder.set(ControlMode.Position, -6600);
    anglePos = angleEncoder.getSelectedSensorPosition();
    SmartDashboard.putNumber("anglePos", anglePos);

    distance = (targetHeight - limelightHeight) / Math.tan(limelightAngle + targetAngle);
    SmartDashboard.putNumber("distance", distance);

    SmartDashboard.putNumber("shooterVelocity", shooter.getSelectedSensorVelocity());

  }


  public void aim() {
    double shootingAngle = 3075.03 - 3.91221 * distance;
    if(shootingAngle<-6600) shootingAngle=-6600;
    if(shootingAngle>3400) shootingAngle=3400;
    angleEncoder.set(ControlMode.Position, shootingAngle);
  }

  public void shootLow() {
    shooter.set(0.3);
    // Timer.delay(0.5);
    angleEncoder.set(ControlMode.Position, -3000);
    Timer.delay(1);
    serializer.set(0.2);
    Timer.delay(1);
    shooter.set(0);
    serializer.set(0);
  }
}

/*
 * List of shooting data:
 * Pos Dis
 * 1000 370
 * 1745 304
 * 2136 270
 * 2146 315
 * 1760     425
 * 
 */