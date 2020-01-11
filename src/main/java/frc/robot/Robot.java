/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// import com.ctre.phoenix;

// import com.ctre.phoenix.motorcontrol.Faults;
// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //private DifferentialDrive myRobot;
  private Joystick xboxController;
  private Joystick stick;

  // WPI_TalonSRX _rghtFront = new WPI_TalonSRX(2);
  // TalonSRX talon1 = new TalonSRX(1);
  // TalonSRX talon2 = new TalonSRX(2);

  public CANSparkMax spark2, spark1;
  private CANPIDController shooterPidController;
  public CANEncoder encoder;
  
  double velocity = -999999;
    double lastEncoderVal;

  NetworkTableInstance ntInst;
  NetworkTable ntShooter;
  NetworkTableEntry ntShooterTargetRate;
  NetworkTableEntry ntShooterVelocity;
  NetworkTableEntry ntShooterMin;
  


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    stick = new Joystick(4);
    ntInst = NetworkTableInstance.getDefault();
    ntShooterMin = ntInst.getEntry("Shooter/Min Velocity");
    ntShooterVelocity = ntInst.getEntry("Shooter/Cur Velocity");
    ntShooterTargetRate = ntInst.getEntry("Shooter/Target Rate");
    //double target = ntShooterTargetRate.getDouble(0);
    
    //SmartDashboard.putNumber("TargetRate", SmartDashboard.getNumber("TargetRate", 0));


    spark2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    spark1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);

    spark2.restoreFactoryDefaults();
    spark1.restoreFactoryDefaults();

    spark1.setSmartCurrentLimit(40, 40);
    spark2.setSmartCurrentLimit(40, 40);

    spark1.setIdleMode(IdleMode.kCoast);
    spark2.setIdleMode(IdleMode.kCoast);

    //spark2.follow(spark1,true);
    //shooterPidController = spark1.getPIDController();
    //SetSparkPID(shooterPidController);



    encoder = spark1.getEncoder();
    encoder.setVelocityConversionFactor(1.0);
  


    //spark1.burnFlash();
    //spark2.burnFlash();

    xboxController = new Joystick(5);
    
  }

  // private void SetSparkPID(CANPIDController pid) {
  //   double kP = 5e-5;  
  //   double kI = 0;
  //   double kD = 0; 
  //   double kIz = 0; 
  //   double kFF = 0; 
  //   double kMaxOutput = 2; 
  //   double kMinOutput = -2;
  //   //double maxRPM = 5700;

  //   pid.setP(kP);
  //   pid.setI(kI);
  //   pid.setD(kD);
  //   pid.setIZone(kIz);
  //   pid.setFF(kFF);
  //   pid.setOutputRange(kMinOutput, kMaxOutput);
  // }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    //myRobot.tankDrive(xboxController.getRawAxis(1), xboxController.getRawAxis(1));


    //SmartDashboard.updateValues();


    boolean useFixedSpeed = xboxController.getRawAxis(2)>0.5;
    double shooterRate = ntShooterTargetRate.getDouble(0); 

    SmartDashboard.putBoolean("useFixedSpeed", useFixedSpeed);

    if(useFixedSpeed) {
      //shooterRate = SmartDashboard.getNumber("TargetRate", 0);
      //SmartDashboard.putNumber("echoTargetRate", shooterRate);
      //shooterPidController.setReference(3200 * -5.0, ControlType.kVelocity);
    } else {
      double stickSpeed = stick.getRawAxis(2)*1.0;
      double shootSpeed;
      if(stick.getRawButton(10)) {  
        //if(encoder.getVelocity()< 2500){
        //  shootSpeed = -0.44+.05;
        //}else{
          shootSpeed= -0.48;
        //}
      } else { 
        shootSpeed = 0;
      }
      SmartDashboard.putNumber("echoStickSpeed", stickSpeed);
      SmartDashboard.putNumber("echoShootSpeed", shootSpeed);
      //shooterPidController.setReference(manualSpeed, ControlType.kVelocity);
      spark1.set(shootSpeed);
      spark2.set(-shootSpeed);
    }

    if(xboxController.getRawButton(3)){
      double v = encoder.getVelocity();
      if(v > velocity){
        velocity = v;
        SmartDashboard.putNumber("Minimum Velocity", (velocity));
      }   
    }
    if(xboxController.getRawButton(4)){
      velocity = -999999;
    }

    SmartDashboard.putNumber("Current Velocity", (encoder.getVelocity()));
    SmartDashboard.updateValues();
}
}



