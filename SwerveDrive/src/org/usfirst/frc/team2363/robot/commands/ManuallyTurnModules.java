package org.usfirst.frc.team2363.robot.commands;

import static org.usfirst.frc.team2363.robot.Robot.drivetrain;
import static org.usfirst.frc.team2363.robot.Robot.oi;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManuallyTurnModules extends Command {

    public ManuallyTurnModules() {
        requires(drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
    	drivetrain.setFrontLeftAngle(oi.getZ());
    	drivetrain.setFrontRightAngle(oi.getZ());
    	drivetrain.setRearLeftAngle(oi.getZ());
    	drivetrain.setRearRightAngle(oi.getZ());
    	
    	SmartDashboard.putNumber("Left Front Position", drivetrain.getFrontLeftAngle());
    	SmartDashboard.putNumber("Right Front Position", drivetrain.getFrontRightAngle());
    	SmartDashboard.putNumber("Left Front Rear", drivetrain.getRearLeftAngle());
    	SmartDashboard.putNumber("Right Rear Position", drivetrain.getRearRightAngle());
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
