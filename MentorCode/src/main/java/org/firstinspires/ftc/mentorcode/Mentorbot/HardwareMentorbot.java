package org.firstinspires.ftc.mentorcode.Mentorbot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
public class HardwareMentorbot
{
    /* Public OpMode members. */
    public DcMotor      leftMotor           = null;
    public DcMotor      rightMotor          = null;
    public DcMotor      shooter             = null;
    public Servo        beaconPusher        = null;
    public ColorSensor  beaconColorSensor   = null;
    public AnalogInput  irLineSensorFront   = null;
    public AnalogInput  irLineSensorBack    = null;
    public ModernRoboticsI2cRangeSensor rangeSensor = null;

    public ElapsedTime autoTimer  = new ElapsedTime();

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    static final double driveGearReduction      = 1;
    static final double robotWidthInches        = 12;

    static final double wheelDiameterInches = 4;
    static final double wheelCircumference = wheelDiameterInches * Math.PI;
    static final double clicksPerRotation = 1440; //declares encoder clicks per full rotation on Tetrix encoder
    static final double beaconPusherRetract = 0;
    static final double beaconPusherSense = 0.2;
    static final double beaconPusherPressButton = 0.4;
    static int countsTurnDownWall = 4000;
    static int countsTurnToCapBall = 8000;
    static int countsToHitCapBall = 8000;
    static int colorVerified = 100;

    double wallScaler = .5;
    double wallDistance = 7;




    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap passhwMap) {
        // save reference to HW Map
        hwMap = passhwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left motor");
        rightMotor  = hwMap.dcMotor.get("right motor");
        shooter = hwMap.dcMotor.get("ball shooter");
        beaconPusher = hwMap.servo.get("beacon pusher");
        beaconColorSensor = hwMap.colorSensor.get("beacon color sensor");
        irLineSensorFront = hwMap.analogInput.get("ir line sensor front");
        irLineSensorBack = hwMap.analogInput.get("ir line sensor back");
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        beaconPusher.setPosition(beaconPusherRetract);
    }


    double countsPerInch () {
        return clicksPerRotation * driveGearReduction / (wheelDiameterInches * Math.PI);
    }

    double countsPerTurnDegree () {
        return 2 * Math.PI * robotWidthInches * countsPerInch() / 360;
    }

    double robotDriveDistanceFromWall() {
        return (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;
    }

}
