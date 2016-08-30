package org.usfirst.frc.team2976.robot.subsystems;

import org.usfirst.frc.team2976.robot.Robot;
import org.usfirst.frc.team2976.robot.RobotMap;
import org.usfirst.frc.team2976.robot.commands.ArmControl;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import util.PIDMain;
import util.PIDSource;

/**
 *
 */
public class Arm extends Subsystem {
	private CANTalon rightArm;
	private CANTalon leftArm;
	
	private Encoder rightArmEncoder, leftArmEncoder;

	public PIDSource rightArmPIDSource;
	public PIDSource leftArmPIDSource;

	public PIDMain rightArmPID;
	public PIDMain leftArmPID;

	private double kp = 0;
	private double ki = 0;
	private double kd = 0;

	public boolean overAmped = false;
	public final double maxCurrent = 15; //TODO: this is a rough estimate

	public Arm() {
		rightArm = new CANTalon(RobotMap.RightArmMotor);
		leftArm = new CANTalon(RobotMap.LeftArmMotor);

		leftArmEncoder = new Encoder(RobotMap.LeftArmEncoderA, RobotMap.LeftArmEncoderB);
		rightArmEncoder = new Encoder(RobotMap.RightArmEncoderA, RobotMap.RightArmEncoderB);

		rightArmPIDSource = new PIDSource() {
			public double getInput() {
				return rightArmEncoder.get();
			}
		};
		leftArmPIDSource = new PIDSource() {
			public double getInput() {
				return leftArmEncoder.get();
			}
		};

		rightArmPID = new PIDMain(rightArmPIDSource, 0, 100, kp, ki, kd);
		leftArmPID = new PIDMain(leftArmPIDSource, 0, 100, kp, ki, kd);
		rightArmPID.isEnabled(true);
		leftArmPID.isEnabled(true);
		
		rightArmPID.setOutputLimits(-0.3, 0.3);
		leftArmPID.setOutputLimits(-0.3, 0.3);
	}

	public void setPosition(double position) {
		rightArmPID.setSetpoint(position);
		leftArmPID.setSetpoint(position);
		setPower(rightArmPID.getOutput(), leftArmPID.getOutput());
	}

	public void setPower(double rightPower, double leftPower) {
		SmartDashboard.putNumber("ArmOutputCurrentAverage",
				(leftArm.getOutputCurrent() + rightArm.getOutputCurrent()) / 2);
		if (rightArm.getOutputCurrent() > maxCurrent || leftArm.getOutputVoltage() > maxCurrent) {
			overAmped = true;
		}
		if (Robot.oi.armStick.getPOV() != -1) {
			overAmped = false;
		}
		if (!overAmped) {
			rightArm.set(rightPower / 3);
			leftArm.set(leftPower / 3);
		} else {
			rightArm.set(0);
			leftArm.set(0);
		}
	}

	public void initDefaultCommand() {
		setDefaultCommand(new ArmControl());
	}
}
