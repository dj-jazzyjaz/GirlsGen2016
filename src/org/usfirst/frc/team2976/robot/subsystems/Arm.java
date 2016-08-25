package org.usfirst.frc.team2976.robot.subsystems;

import org.usfirst.frc.team2976.robot.RobotMap;
import org.usfirst.frc.team2976.robot.commands.ArmControl;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import util.PIDMain;
import util.PIDSource;

/**
 *
 */
public class Arm extends Subsystem {
	public CANTalon rightArm;
	public CANTalon leftArm;
	public Talon roller;
	private Encoder rightArmEncoder, leftArmEncoder;

	public PIDSource rightArmPIDSource;
	public PIDSource leftArmPIDSource;

	public PIDMain rightArmPID;
	public PIDMain leftArmPID;

	public Arm()	{
		double kp = 0.1;
		double ki = 0;
		double kd = 0;
		
    	rightArm =  new CANTalon(RobotMap.RightArmMotor);
    	leftArm = new CANTalon (RobotMap.LeftArmMotor);
    	leftArmEncoder = new Encoder(RobotMap.LeftArmEncoderA,RobotMap.LeftArmEncoderB);
    	rightArmEncoder = new Encoder(RobotMap.RightArmEncoderA,RobotMap.RightArmEncoderB); 
    	
    	rightArmPIDSource = new PIDSource()	{
			public double getInput() {
				return rightArmEncoder.get();
			}
		};
		leftArmPIDSource = new PIDSource()	{
			public double getInput() {
				return leftArmEncoder.get();
			}
		};
		
		rightArmPID = new PIDMain(rightArmPIDSource, 0, 100, kp, ki, kd);
		leftArmPID = new PIDMain(leftArmPIDSource, 0, 100, kp, ki, kd);
		rightArmPID.isEnabled(true);
		leftArmPID.isEnabled(true);
	}
	
	public void setPosition(double position)	{
		rightArmPID.setSetpoint(position);
		leftArmPID.setSetpoint(position);
		setPower(rightArmPID.getOutput(),leftArmPID.getOutput());
	}
	public void setPower(double rightPower, double leftPower)	{
		rightArm.set(rightPower);
		leftArm.set(leftPower);
	}
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ArmControl());
	}
}
