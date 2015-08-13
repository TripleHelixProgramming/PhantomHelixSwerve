package org.usfirst.frc.team2363.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	private final int Y_AXIS = 1;
	private final int X_AXIS = 0;
	private final int Z_AXIS = 2;
	
	private Joystick gameController;
	
	public OI() {
		gameController = new Joystick(0);
	}
	
	/**
	 * Get the forward(y) magnitude from the game controller
	 * @return the magnitude of y from the game controller
	 */
	public double getY() {
		return gameController.getRawAxis(Y_AXIS);
	}
	
	/**
	 * Get the left/right(x) strafing magnitude from the game controller
	 * @return the magnitude of x from the game controller
	 */
	public double getX() {
		return gameController.getRawAxis(X_AXIS);
	}
	
	/**
	 * Get the rotation(z) magnitude from the game controller
	 * @return the magnitude of z from the game controller
	 */
	public double getZ() {
		return gameController.getRawAxis(Z_AXIS);
	}
}

