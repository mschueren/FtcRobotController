package org.firstinspires.ftc.mentorcode.Mentorbot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* MentorBot first Autonomous Program
 * Designed for Red side of Field
 */

@Autonomous(name = "Red Follow Wall", group = "Mentorbot")
@Disabled

public class MentorRedAuto1 extends OpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor shooter;
    Servo beaconPusher;
    ColorSensor beaconColorSensor;
    OpticalDistanceSensor odsWallFollower;
    ModernRoboticsI2cRangeSensor rangeSensor;
    AnalogInput irLineSensorFront, irLineSensorBack;

    private ElapsedTime autoTimer = new ElapsedTime();

    static double wheelDiameter = 4; //in inches
    static double wheelCircumference = wheelDiameter * Math.PI; //Calculating Circumference
    static double ClicksPerRotation = 1440; //declares encoder clicks per full rotation on Tetrix encoder
    static double beaconPusherRetract = 0;
    static double beaconPusherSense = 0.2;
    static double beaconPusherPressButton = 0.4;
    static int countsTurnTowardWall = 5000;
    static int countsToWall = 4000;
    static int countsTurnDownWall = 4000;
    static int countsTurnToCapBall = 8000;
    static int countsToHitCapBall = 8000;
    static int colorVerified = 100;

    double rightPower = 0;
    double leftPower = 0;
    double wallScaler = .5;
    double wallDistance = 7;
    int rightTarget = 0;
    int leftTarget = 0;
    int colorCheck = 0;

    State programState;

    public enum State {
        SCORE_PARTICLE,
        TURN_TO_WALL,
        FORWARD_TO_WALL,
        TURN_DOWN_WALL,
        FOLLOW_WALL_TO_BEACON1A,
        CHECK_BEACON1A,
        FOLLOW_WALL_TO_BEACON1B,
        CHECK_BEACON1B,
        FOLLOW_WALL_TO_BEACON2A,
        CHECK_BEACON2A,
        FOLLOW_WALL_TO_BEACON2B,
        CHECK_BEACON2B,
        TURN_TO_CAP_BALL,
        FORWARD_TO_HIT_CAP_BALL,
        END

    }


    public MentorRedAuto1() {
    } //Constructor is empty

    @Override
    public void init() {

        rightMotor = hardwareMap.dcMotor.get("rightmotor");
        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        shooter = hardwareMap.dcMotor.get("armExtend");
        beaconPusher = hardwareMap.servo.get("beaconpusher");
        beaconColorSensor = hardwareMap.colorSensor.get("beaconcolorsensor");
        odsWallFollower = hardwareMap.opticalDistanceSensor.get("odswallfollower");
        irLineSensorFront = hardwareMap.analogInput.get("irlinesensorfront");
        irLineSensorBack = hardwareMap.analogInput.get("irlinesensorback");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        beaconPusher.setPosition(beaconPusherRetract);

        programState = State.SCORE_PARTICLE;


    }

    @Override
    public void init_loop() {

        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        autoTimer.reset();

        telemetry.addData("1. Right Motor Pos", rightMotor.getCurrentPosition());
        telemetry.addData("2. Left Motor Pos", leftMotor.getCurrentPosition());
        telemetry.addData("3. Beacon Color Sensor", beaconColorSensor.alpha());
        telemetry.addData("4. Opt Dis Sensor Value", odsWallFollower.getRawLightDetected());
        telemetry.addData("5. Front IR Value", irLineSensorFront.getVoltage());
        telemetry.addData("6. Back IR Value", irLineSensorBack.getVoltage());
        telemetry.addData("7. Range Sensor Inch", rangeSensor.getDistance(DistanceUnit.INCH));
    }

    @Override
    public void loop() {

        switch (programState) {

            case SCORE_PARTICLE:
                shooter.setPower(1.0);

                if(autoTimer.milliseconds()>1000){
                    programState= State.TURN_TO_WALL;
                    rightTarget = rightMotor.getCurrentPosition() + countsTurnTowardWall;
                    rightMotor.setTargetPosition(rightTarget);
                }

                break;

            case TURN_TO_WALL:
                rightMotor.setPower(1.0);
                leftMotor.setPower(0);

                if (Math.abs(rightMotor.getCurrentPosition()-rightTarget) < 2) {
                    programState = State.FORWARD_TO_WALL;
                    rightTarget = rightMotor.getCurrentPosition() + countsToWall;
                    leftTarget = leftMotor.getCurrentPosition() + countsToWall;

                    rightMotor.setTargetPosition(rightTarget);
                    leftMotor.setTargetPosition(leftTarget);
                }

                break;

            case FORWARD_TO_WALL:
                rightMotor.setPower(1.0);
                leftMotor.setPower(1.0);

                if (Math.abs(rightMotor.getCurrentPosition() - rightTarget) < 2  &&
                        Math.abs(leftMotor.getCurrentPosition() - leftTarget) < 2) {
                    programState = State.TURN_DOWN_WALL;
                    leftTarget = leftMotor.getCurrentPosition() + countsTurnDownWall;

                    leftMotor.setTargetPosition(leftTarget);
                }
                break;

            case TURN_DOWN_WALL:
                rightMotor.setPower(0);
                leftMotor.setPower(1.0);

                if (Math.abs(leftMotor.getCurrentPosition()- leftTarget) < 2) {
                    programState = State.FOLLOW_WALL_TO_BEACON1A;
                    rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                break;

            case FOLLOW_WALL_TO_BEACON1A:
                rightPower = 0.5 - (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;
                leftPower =  0.5 + (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;

                rightPower = Range.clip(rightPower, -1, 1);
                leftPower = Range.clip(leftPower, -1, 1);

                rightMotor.setPower(rightPower);
                leftMotor.setPower(leftPower);

                if (irLineSensorFront.getVoltage() > 0.9) {
                    programState = State.CHECK_BEACON1A;
                    rightMotor.setPower(0);
                    leftMotor.setPower(0);
                }
                break;

            case CHECK_BEACON1A:
                beaconPusher.setPosition(beaconPusherSense);
                autoTimer.reset();

                if (beaconColorSensor.alpha() > 5) {
                    colorCheck = beaconColorSensor.red() - beaconColorSensor.blue();

                    if (colorCheck > colorVerified * 2) {
                        beaconPusher.setPosition(beaconPusherRetract);
                        programState = State.FOLLOW_WALL_TO_BEACON2A;
                    } else if (colorCheck > colorVerified) {
                        beaconPusher.setPosition(beaconPusherPressButton);
                    } else if (colorCheck < -colorVerified) {
                        beaconPusher.setPosition(beaconPusherRetract);
                        programState = State.FOLLOW_WALL_TO_BEACON1B;
                    }
                }
                break;

            case FOLLOW_WALL_TO_BEACON1B:
                rightPower = 0.5 - (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;
                leftPower =  0.5 + (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;

                rightPower = Range.clip(rightPower, -1, 1);
                leftPower = Range.clip(leftPower, -1, 1);

                rightMotor.setPower(rightPower);
                leftMotor.setPower(leftPower);

                if (irLineSensorBack.getVoltage() > 0.9 && autoTimer.milliseconds() > 500) {
                    programState = State.CHECK_BEACON1B;
                    rightMotor.setPower(0);
                    leftMotor.setPower(0);
                    colorCheck = 0;
                }
                break;

            case CHECK_BEACON1B:
                beaconPusher.setPosition(beaconPusherPressButton);
                autoTimer.reset();

                if (beaconColorSensor.alpha() > 5) {
                    colorCheck = beaconColorSensor.red() - beaconColorSensor.blue();

                    if (colorCheck > colorVerified) {
                        beaconPusher.setPosition(beaconPusherRetract);
                        programState = State.FOLLOW_WALL_TO_BEACON2A;
                        colorCheck = 0;
                    }
                }
                break;

            case FOLLOW_WALL_TO_BEACON2A:
                rightPower = 0.5 - (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;
                leftPower =  0.5 + (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;

                rightPower = Range.clip(rightPower, -1, 1);
                leftPower = Range.clip(leftPower, -1, 1);

                rightMotor.setPower(rightPower);
                leftMotor.setPower(leftPower);

                if (irLineSensorFront.getVoltage() > 0.9 && autoTimer.milliseconds() > 500) {
                    programState = State.CHECK_BEACON2A;
                    rightMotor.setPower(0);
                    leftMotor.setPower(0);
                    colorCheck = 0;
                }
                break;

            case CHECK_BEACON2A:
                beaconPusher.setPosition(beaconPusherSense);
                autoTimer.reset();

                if (beaconColorSensor.alpha() > 5) {
                    colorCheck = beaconColorSensor.red() - beaconColorSensor.blue();

                    if (colorCheck > colorVerified * 2) {
                        beaconPusher.setPosition(beaconPusherRetract);
                        programState = State.TURN_TO_CAP_BALL;

                        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        rightTarget = rightMotor.getCurrentPosition();
                        leftTarget = leftMotor.getCurrentPosition() + countsTurnToCapBall;

                        rightMotor.setTargetPosition(rightTarget);
                        leftMotor.setTargetPosition(leftTarget);

                    } else if (colorCheck > colorVerified) {
                        beaconPusher.setPosition(beaconPusherPressButton);
                    } else if (colorCheck < -colorVerified) {
                        beaconPusher.setPosition(beaconPusherRetract);
                        programState = State.FOLLOW_WALL_TO_BEACON2B;
                    }
                }
                break;

            case FOLLOW_WALL_TO_BEACON2B:
                rightPower = 0.5 - (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;
                leftPower =  0.5 + (wallDistance - rangeSensor.getDistance(DistanceUnit.INCH)) * wallScaler;

                rightPower = Range.clip(rightPower, -1, 1);
                leftPower = Range.clip(leftPower, -1, 1);

                rightMotor.setPower(rightPower);
                leftMotor.setPower(leftPower);

                if (irLineSensorBack.getVoltage() > 0.9 && autoTimer.milliseconds() > 500) {
                    programState = State.CHECK_BEACON2B;
                    rightMotor.setPower(0);
                    leftMotor.setPower(0);
                    colorCheck = 0;
                }
                break;

            case CHECK_BEACON2B:
                beaconPusher.setPosition(beaconPusherPressButton);
                autoTimer.reset();

                if (beaconColorSensor.alpha() > 5) {
                    colorCheck = beaconColorSensor.red() - beaconColorSensor.blue();

                    if (colorCheck > colorVerified) {
                        beaconPusher.setPosition(beaconPusherRetract);
                        programState = State.TURN_TO_CAP_BALL;

                        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        rightTarget = rightMotor.getCurrentPosition();
                        leftTarget = leftMotor.getCurrentPosition() + countsTurnToCapBall;

                        rightMotor.setTargetPosition(rightTarget);
                        leftMotor.setTargetPosition(leftTarget);
                    }
                }
                break;

            case TURN_TO_CAP_BALL:
                rightMotor.setPower(0);
                leftMotor.setPower(1.0);

                if (Math.abs(leftMotor.getCurrentPosition()- leftTarget) < 2) {
                    programState = State.FORWARD_TO_HIT_CAP_BALL;

                    rightTarget = rightMotor.getCurrentPosition() + countsToHitCapBall;
                    leftTarget = leftMotor.getCurrentPosition() + countsToHitCapBall;

                    rightMotor.setTargetPosition(rightTarget);
                    leftMotor.setTargetPosition(leftTarget);
                }
                break;

            case FORWARD_TO_HIT_CAP_BALL:
                rightMotor.setPower(1.0);
                leftMotor.setPower(1.0);

                if (Math.abs(rightMotor.getCurrentPosition() - rightTarget) < 2  &&
                        Math.abs(leftMotor.getCurrentPosition() - leftTarget) < 2) {
                    programState = State.END;
                }
                break;

            case END:
                rightMotor.setPower(0);
                leftMotor.setPower(0);
                break;

        }


    }

    @Override
    public void stop() {
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}
