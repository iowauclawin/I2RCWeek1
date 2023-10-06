// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase 
{
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP); //This is a constructor, which creates an object in the AHRS class called navx

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain"); //This titles the variable DTTab into DriveTrain
  private GenericEntry LeftVoltage = DTTab.add("Left Voltage", 0.0).getEntry(); //Updates DTTab with Left Voltage with the value of 0.0 and stores the return value of getEntry into LeftVoltage
  private GenericEntry RightVoltage = DTTab.add("Right Voltage", 0.0).getEntry(); //Updates DTTab with Right Voltage with the value of 0.0 and stores the return value of getEntry into RightVoltage

  /** Creates a new DriveTrain */
  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort); //Creating an object by using a constructor for leftDriveTalon
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort); //Same as above except for rightDriveTalon
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast); //Sets leftDriveTalon to neutral
    rightDriveTalon.setNeutralMode(NeutralMode.Coast); //Sets rightDriveTalon to neutral

    leftDriveTalon.setInverted(true); //Make sure the leftDriveTalon is inverted compared to the rightDriveTalon so it can move forward and backward correctly
    rightDriveTalon.setInverted(false); //Make sure the rightDriveTalon is inverted compared to the leftDriveTalon so it can move forward and backward correctly

    leftDriveTalon.setSensorPhase(true); //Sets the sensors for leftDriveTalon to true
    rightDriveTalon.setSensorPhase(true); //Sets the sensors for rightDriveTalon to true

    leftDriveTalon.configFactoryDefault(); //resets leftDriveTalon
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //Sets up encoder to track rotation of motor
    rightDriveTalon.configFactoryDefault(); //resets rightDriveTalon
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) { //This will drive the robot with a certain speed
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10); //Sets sensor position to 0,0,10
    rightDriveTalon.setSelectedSensorPosition(0,0,10); //Sets sensor position to 0,0,10
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }
 
  public double getAngle(){ //Gets the robot's current angle
    return navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());
    SmartDashboard.putNumber("Ticks", getTicks());
    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent());
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());

  }
}