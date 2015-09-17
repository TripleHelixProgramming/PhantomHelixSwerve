package org.usfirst.frc.team2363.robot.subsystems;

import org.usfirst.frc.team2363.robot.commands.NonFieldCentricDrive;

import com.kauailabs.nav6.frc.IMU;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drivetrain extends Subsystem {
	
	private SwerveModule frontLeft = new SwerveModule(8, 2, 230, false);
	private SwerveModule frontRight = new SwerveModule(1, 4, 420, false);
	private SwerveModule rearLeft = new SwerveModule(5, 10, 950, false);
	private SwerveModule rearRight = new SwerveModule(6, 11, 200, false);
    
  	private IMU imu;
    
    private final double WHEEL_BASE = 24;
    private final double TRACK_WIDTH = 8;
    private final double R = Math.sqrt(WHEEL_BASE * WHEEL_BASE + TRACK_WIDTH * TRACK_WIDTH);
    
    public Drivetrain() {
		try {
			SerialPort serialPort = new SerialPort(57600, SerialPort.Port.kMXP);
			byte updateRateHz = 50;
			imu = new IMU(serialPort, updateRateHz);
			imu.zeroYaw();
		} catch (Exception ex) {
			ex.printStackTrace();
		}
    }

    @Override
    public void initDefaultCommand() {
    	setDefaultCommand(new NonFieldCentricDrive());
    }
    
    /**
     * Drive the swerve modules based on the input
     * @param y forward power
     * @param x strafing power
     * @param z rotational power
     * @param fieldCentric true if the robot should drive in field centric mode
     */
    public void drive(double y, double x, double z, boolean fieldCentric) {
    	if (fieldCentric) {
    		drive(y * Math.cos(getFacing()) + x * Math.sin(getFacing()),
    				-y * Math.sin(getFacing()) + x * Math.cos(getFacing()),
    				z);
    	} else {
	    	drive(y, x, z);
    	}
    }
    
    /**
     * Drive the swerve modules based on the input
     * @param y forward power
     * @param x strafing power
     * @param z rotational power
     */
    public void drive(double y, double x, double z) {
    	double a = x - z * (WHEEL_BASE / R);
    	double b = x + z * (WHEEL_BASE / R);
    	double c = y - z * (TRACK_WIDTH / R);
    	double d = y + z * (TRACK_WIDTH / R);
    	
    	//Calculate module angles
    	double fla = Math.atan2(b, d) * 180.0 / Math.PI;
    	double fra = Math.atan2(b, c) * 180.0 / Math.PI;
    	double rla = Math.atan2(a, d) * 180.0 / Math.PI;
    	double rra = Math.atan2(a, c) * 180.0 / Math.PI;
    	
    	setFrontLeftAngle(fla);
    	SmartDashboard.putNumber("Front Left Angle", fla);
    	setFrontRightAngle(fra);
    	SmartDashboard.putNumber("Front Right Angle", fra);
    	setRearLeftAngle(rla);
    	SmartDashboard.putNumber("Rear Left Angle", rla);
    	setRearRightAngle(rra);
    	SmartDashboard.putNumber("Rear Right Angle", rra);
    	
    	//Calculate wheel speeds
    	double frontRightWheelSpeed = Math.sqrt(b * b + d * d);
    	double frontLeftWheelSpeed = Math.sqrt(b * b + c * c);
    	double rearRightWheelSpeed = Math.sqrt(a * a + d * d);
    	double rearLeftWheelSpeed = Math.sqrt(a * a + c * c);
    	
    	//Normalize wheel speeds
    	double max = frontLeftWheelSpeed;
    	max = frontRightWheelSpeed > max ? frontRightWheelSpeed : max;
    	max = rearLeftWheelSpeed > max ? rearLeftWheelSpeed : max;
    	max = rearRightWheelSpeed > max ? rearRightWheelSpeed : max;
    	
    	if (max > 1) {
    		frontLeftWheelSpeed /= max;
    		frontLeftWheelSpeed /= max;
    		frontRightWheelSpeed /= max;
    		rearLeftWheelSpeed /= max;
    		rearRightWheelSpeed /= max;
    	}
    	
    	setFrontLeftSpeed(frontLeftWheelSpeed);
    	SmartDashboard.putNumber("Front Left Speed", frontLeftWheelSpeed);
    	setFrontRightSpeed(frontRightWheelSpeed);
    	SmartDashboard.putNumber("Front Right Speed", frontRightWheelSpeed);
    	setRearLeftSpeed(rearLeftWheelSpeed);
    	SmartDashboard.putNumber("Rear Left Speed", rearLeftWheelSpeed);
    	setRearRightSpeed(rearRightWheelSpeed);
    	SmartDashboard.putNumber("Rear Right Speed", rearRightWheelSpeed);
    }
    
    
    
    /**
     * Set the angle of the front left wheel
     * @param angle the angle to turn the wheel to
     */
    public void setFrontLeftAngle(double angle) {
    	frontLeft.setAngle(angle);
    }
    
    /**
     * Get the current angle of the front left wheel
     * @return the angle the wheel is currently facing
     */
    public double getFrontLeftAngle() {
    	return frontLeft.getAngle();
    }
    
    /**
     * Set the angle of the front right wheel
     * @param angle the angle to turn the wheel to
     */
    public void setFrontRightAngle(double angle) {
    	frontRight.setAngle(angle);
    }
    
    /**
     * Get the current angle of the front right wheel
     * @return the angle the wheel is currently facing
     */
    public double getFrontRightAngle() {
    	return frontRight.getAngle();
    }
    
    /**
     * Set the angle of the rear left wheel
     * @param angle the angle to turn the wheel to
     */
    public void setRearLeftAngle(double angle) {
    	rearLeft.setAngle(angle);
    }
    
    /**
     * Get the current angle of the rear left wheel
     * @return the angle the wheel is currently facing
     */
    public double getRearLeftAngle() {
    	return rearLeft.getAngle();
    }
    
    /**
     * Set the angle of the rear right wheel
     * @param angle the angle to turn the wheel to
     */
    public void setRearRightAngle(double angle) {
    	rearRight.setAngle(angle);
    }
    
    /**
     * Get the current angle of the rear right wheel
     * @return the angle the wheel is currently facing
     */
    public double getRearRightAngle() {
    	return rearRight.getAngle();
    }
    
    /**
     * Set the speed of the front left wheel
     * @param speed the speed to turn the wheel at
     */
    public void setFrontLeftSpeed(double speed) {
    	frontLeft.setSpeed(speed);
    }
    
    /**
     * Get the current speed of the front left wheel
     * @return the current speed of the wheel
     */
    public double getFrontLeftSpeed() {
    	return frontLeft.getSpeed();
    }
    
    /**
     * Set the speed of the front right wheel
     * @param speed the speed to turn the wheel at
     */
    public void setFrontRightSpeed(double speed) {
    	frontRight.setSpeed(speed);
    }
    
    /**
     * Get the current speed of the front right wheel
     * @return the current speed of the wheel
     */
    public double getFrontRightSpeed() {
    	return frontRight.getSpeed();
    }
    
    /**
     * Set the speed of the rear left wheel
     * @param speed the speed to turn the wheel at
     */
    public void setRearLeftSpeed(double speed) {
    	rearLeft.setSpeed(speed);
    }
    
    /**
     * Get the current speed of the rear left wheel
     * @return the current speed of the wheel
     */
    public double getRearLeftSpeed() {
    	return rearLeft.getSpeed();
    }
    
    /**
     * Set the speed of the rear right wheel
     * @param speed the speed to turn the wheel at
     */
    public void setRearRightSpeed(double speed) {
    	rearRight.setSpeed(speed);
    }
    
    /**
     * Get the current speed of the rear right wheel
     * @return the current speed of the wheel
     */
    public double getRearRightSpeed() {
    	return rearRight.getSpeed();
    }
    
    public double getFacing() {
    	return (imu.getYaw() + 270) % 360 - 180;
    }
}

