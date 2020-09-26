package org.firstinspires.ftc.examplecode.Examplebot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 *
 */
public class ExampleHardwareExamplebot
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime runTime  = new ElapsedTime();

    static final double     countsPerMotorRev       = 1440;
    static final double     wheelDiameterInches     = 4;
    static final double     driveGearReduction      = 1;
    static final double     robotWidthInches        = 12;

    /* Constructor */
    public ExampleHardwareExamplebot() {
    }

    double countsPerInch () {
        return countsPerMotorRev * driveGearReduction / (wheelDiameterInches * Math.PI);
    }

    double countsPerTurnDegree () {
        return 2 * Math.PI * robotWidthInches * countsPerInch() / 360;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap passhwMap) {
        // save reference to HW Map
        hwMap = passhwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left motor");
        rightMotor  = hwMap.dcMotor.get("right motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
