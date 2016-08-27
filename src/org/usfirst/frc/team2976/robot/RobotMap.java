package org.usfirst.frc.team2976.robot;
/**
 * @author NeilHazra
 * 
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {
	//Drive Motors
	public static final int RightFrontDriveMotor = 3; 
	public static final int LeftFrontDriveMotor = 2; 
	public static final int RightBackDriveMotor = 4; 
	public static final int LeftBackDriveMotor = 1;
	
	//Arm Motors
	public static final int RightArmMotor = 6; 
	public static final int LeftArmMotor = 5;
	
	//Encoders for Arm PID
	public static final int RightArmEncoderA = 2;
	public static final int RightArmEncoderB = 3;
	public static final int LeftArmEncoderA = 0;
	public static final int LeftArmEncoderB = 1;
	
	//Roller
	public static final int rollerMotorID = 6; 
	
	//Gyro for Drive Straight
	public static final int Gyro = 1;
	
	//Encoders for drive straight
	public static final int RightDriveEncoderA = 0; //FIXME
	public static final int RightDriveEncoderB = 0; //FIXME
	public static final int LeftDriveEncoderA = 5; 
	public static final int LeftDriveEncoderB = 7; 
}
