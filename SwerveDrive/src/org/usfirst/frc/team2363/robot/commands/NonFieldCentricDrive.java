package org.usfirst.frc.team2363.robot.commands;

import static org.usfirst.frc.team2363.robot.Robot.drivetrain;
import static org.usfirst.frc.team2363.robot.Robot.oi;

import edu.wpi.first.wpilibj.command.Command;

public class NonFieldCentricDrive extends Command {

    public NonFieldCentricDrive() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
    	double y = -oi.getY();
    	double x = oi.getX();
    	double z = oi.getZ();
    	
    	if (y > 0.-0.1 && y < 0.1) {
    		y = 0;
    	}
    	
    	if (x > 0.-0.1 && x < 0.1) {
    		x = 0;
    	}
    	
    	if (z > 0.-0.1 && z < 0.1) {
    		z = 0;
    	}
    	
    	if (x == 0 && y == 0 & z == 0) {
    		drivetrain.setFrontLeftAngle(0);
    		drivetrain.setFrontRightAngle(0);
    		drivetrain.setRearLeftAngle(0);
    		drivetrain.setRearRightAngle(0);
    	} else {
    		drivetrain.drive(y, x, z);
    	}
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
