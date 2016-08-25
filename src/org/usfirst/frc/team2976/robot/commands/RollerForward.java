package org.usfirst.frc.team2976.robot.commands;

import org.usfirst.frc.team2976.robot.OI;
import org.usfirst.frc.team2976.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RollerForward extends Command {

    public RollerForward() {
    	requires(Robot.roller);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.roller.setRollerPower(0.3);
    	/*
    	if(Robot.oi.armStick.getRawButton(OI.Button.A.getBtnNumber()))	{
    		Robot.roller.setRollerPower(0.3);
    	}	else if (Robot.oi.armStick.getRawButton(OI.Button.B.getBtnNumber())){
    		Robot.roller.setRollerPower(-0.3);
    	}	else	{
    		Robot.roller.setRollerPower(0);
    	}
    	*/
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
