package frc.swervelib.math;

public class FOC {

    /*
    * @param currentAngle the angle of the robot as measured by the pidgeon from its starting position 
    * @param joyForward the joystick raw value for movement forwards and backwards
    * @param joyStrafe the joystick raw value from the strafing stick
    */
    public static double[] toFieldOriented(double joyForward, double joyStrafe, double currentAngle) {
        double[] translated = {};
        translated[1] = joyForward * Math.cos(currentAngle) + joyStrafe * Math.sin(currentAngle);
        translated[2] = joyStrafe * Math.cos(currentAngle) - joyForward * Math.sin(currentAngle);
        return translated;
    }
}
