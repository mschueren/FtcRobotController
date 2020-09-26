package org.firstinspires.ftc.mentorcode.Mentorbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/* MentorBot first Autonomous Program
 * Designed for Red side of Field
 */

@Autonomous(name = "Red Auto Ramp", group = "Mentorbot")
@Disabled

public class MentorbotRedAutoRamp extends OpMode {

    HardwareMentorbot robot = new HardwareMentorbot();

    double rightPower = 0;
    double leftPower = 0;
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


    public MentorbotRedAutoRamp() {
    } //Constructor is empty

    @Override
    public void init() {
        robot.init(hardwareMap);
        programState = State.SCORE_PARTICLE;
    }

    @Override
    public void init_loop() {
        robot.autoTimer.reset();

        telemetry.addData("1. Right Motor Pos", robot.rightMotor.getCurrentPosition());
        telemetry.addData("2. Left Motor Pos", robot.leftMotor.getCurrentPosition());
        telemetry.addData("3. Beacon Color Sensor", robot.beaconColorSensor.alpha());
        telemetry.addData("5. Front IR Value", robot.irLineSensorFront.getVoltage());
        telemetry.addData("6. Back IR Value", robot.irLineSensorBack.getVoltage());
        telemetry.addData("7. Range Sensor Inch", robot.rangeSensor.getDistance(DistanceUnit.INCH));
    }

    @Override
    public void start() {
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightTarget = (int) (24 * robot.countsPerInch());
        leftTarget = (int) (24 * robot.countsPerInch());
        robot.rightMotor.setTargetPosition(rightTarget + robot.rightMotor.getCurrentPosition());
        robot.leftMotor.setTargetPosition(leftTarget + robot.leftMotor.getCurrentPosition());
    }


    @Override
    public void loop() {

        switch (programState) {

            case SCORE_PARTICLE:
                robot.shooter.setPower(1.0);

                if(robot.autoTimer.milliseconds()>1000){
                    programState= State.TURN_TO_WALL;
                    rightTarget = robot.rightMotor.getCurrentPosition() + (int)(45 * robot.countsPerTurnDegree());
                    robot.rightMotor.setTargetPosition(rightTarget);
                }

                break;

            case TURN_TO_WALL:
                robot.rightMotor.setPower(1.0);
                robot.leftMotor.setPower(0);

                if (Math.abs(robot.rightMotor.getCurrentPosition()-rightTarget) < 2) {
                    programState = State.FORWARD_TO_WALL;
                    rightTarget = robot.rightMotor.getCurrentPosition() + (int)(48 * robot.countsPerInch());
                    leftTarget = robot.leftMotor.getCurrentPosition() + (int)(48 * robot.countsPerInch());

                    robot.rightMotor.setTargetPosition(rightTarget);
                    robot.leftMotor.setTargetPosition(leftTarget);
                }

                break;

            case FORWARD_TO_WALL:
                robot.rightMotor.setPower(1.0);
                robot.leftMotor.setPower(1.0);

                if (Math.abs(robot.rightMotor.getCurrentPosition() - rightTarget) < 2  &&
                        Math.abs(robot.leftMotor.getCurrentPosition() - leftTarget) < 2) {
                    programState = State.TURN_DOWN_WALL;
                    leftTarget = robot.leftMotor.getCurrentPosition() + (int)(45 * robot.countsPerTurnDegree());

                    robot.leftMotor.setTargetPosition(leftTarget);
                }
                break;

            case TURN_DOWN_WALL:
                robot.rightMotor.setPower(0);
                robot.leftMotor.setPower(1.0);

                if (Math.abs(robot.leftMotor.getCurrentPosition()- leftTarget) < 2) {
                    programState = State.FOLLOW_WALL_TO_BEACON1A;
                    robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                break;

            case FOLLOW_WALL_TO_BEACON1A:
                rightPower = 0.5 - robot.robotDriveDistanceFromWall();
                leftPower =  0.5 + robot.robotDriveDistanceFromWall();

                rightPower = Range.clip(rightPower, -1, 1);
                leftPower = Range.clip(leftPower, -1, 1);

                robot.rightMotor.setPower(rightPower);
                robot.leftMotor.setPower(leftPower);

                if (robot.irLineSensorFront.getVoltage() > 0.9) {
                    programState = State.CHECK_BEACON1A;
                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);
                }
                break;

            case CHECK_BEACON1A:
                robot.beaconPusher.setPosition(robot.beaconPusherSense);
                robot.autoTimer.reset();

                if (robot.beaconColorSensor.alpha() > 5) {
                    colorCheck = robot.beaconColorSensor.red() - robot.beaconColorSensor.blue();

                    if (colorCheck > robot.colorVerified * 2) {
                        robot.beaconPusher.setPosition(robot.beaconPusherRetract);
                        programState = State.FOLLOW_WALL_TO_BEACON2A;
                    } else if (colorCheck > robot.colorVerified) {
                        robot.beaconPusher.setPosition(robot.beaconPusherPressButton);
                    } else if (colorCheck < -robot.colorVerified) {
                        robot.beaconPusher.setPosition(robot.beaconPusherRetract);
                        programState = State.FOLLOW_WALL_TO_BEACON1B;
                    }
                }
                break;

            case FOLLOW_WALL_TO_BEACON1B:
                rightPower = 0.5 - robot.robotDriveDistanceFromWall();
                leftPower =  0.5 + robot.robotDriveDistanceFromWall();

                rightPower = Range.clip(rightPower, -1, 1);
                leftPower = Range.clip(leftPower, -1, 1);

                robot.rightMotor.setPower(rightPower);
                robot.leftMotor.setPower(leftPower);

                if (robot.irLineSensorBack.getVoltage() > 0.9 && robot.autoTimer.milliseconds() > 500) {
                    programState = State.CHECK_BEACON1B;
                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);
                    colorCheck = 0;
                }
                break;

            case CHECK_BEACON1B:
                robot.beaconPusher.setPosition(robot.beaconPusherPressButton);
                robot.autoTimer.reset();

                if (robot.beaconColorSensor.alpha() > 5) {
                    colorCheck = robot.beaconColorSensor.red() - robot.beaconColorSensor.blue();

                    if (colorCheck > robot.colorVerified) {
                        robot.beaconPusher.setPosition(robot.beaconPusherRetract);
                        programState = State.FOLLOW_WALL_TO_BEACON2A;
                        colorCheck = 0;
                    }
                }
                break;

            case FOLLOW_WALL_TO_BEACON2A:
                rightPower = 0.5 - robot.robotDriveDistanceFromWall();
                leftPower =  0.5 + robot.robotDriveDistanceFromWall();

                rightPower = Range.clip(rightPower, -1, 1);
                leftPower = Range.clip(leftPower, -1, 1);

                robot.rightMotor.setPower(rightPower);
                robot.leftMotor.setPower(leftPower);

                if (robot.irLineSensorFront.getVoltage() > 0.9 && robot.autoTimer.milliseconds() > 500) {
                    programState = State.CHECK_BEACON2A;
                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);
                    colorCheck = 0;
                }
                break;

            case CHECK_BEACON2A:
                robot.beaconPusher.setPosition(robot.beaconPusherSense);
                robot.autoTimer.reset();

                if (robot.beaconColorSensor.alpha() > 5) {
                    colorCheck = robot.beaconColorSensor.red() - robot.beaconColorSensor.blue();

                    if (colorCheck > robot.colorVerified * 2) {
                        robot.beaconPusher.setPosition(robot.beaconPusherRetract);
                        programState = State.TURN_TO_CAP_BALL;

                        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        rightTarget = robot.rightMotor.getCurrentPosition();
                        leftTarget = robot.leftMotor.getCurrentPosition() + (int)(135 * robot.countsPerTurnDegree());

                        robot.rightMotor.setTargetPosition(rightTarget);
                        robot.leftMotor.setTargetPosition(leftTarget);

                    } else if (colorCheck > robot.colorVerified) {
                        robot.beaconPusher.setPosition(robot.beaconPusherPressButton);
                    } else if (colorCheck < -robot.colorVerified) {
                        robot.beaconPusher.setPosition(robot.beaconPusherRetract);
                        programState = State.FOLLOW_WALL_TO_BEACON2B;
                    }
                }
                break;

            case FOLLOW_WALL_TO_BEACON2B:
                rightPower = 0.5 - robot.robotDriveDistanceFromWall();
                leftPower =  0.5 + robot.robotDriveDistanceFromWall();

                rightPower = Range.clip(rightPower, -1, 1);
                leftPower = Range.clip(leftPower, -1, 1);

                robot.rightMotor.setPower(rightPower);
                robot.leftMotor.setPower(leftPower);

                if (robot.irLineSensorBack.getVoltage() > 0.9 && robot.autoTimer.milliseconds() > 500) {
                    programState = State.CHECK_BEACON2B;
                    robot.rightMotor.setPower(0);
                    robot.leftMotor.setPower(0);
                    colorCheck = 0;
                }
                break;

            case CHECK_BEACON2B:
                robot.beaconPusher.setPosition(robot.beaconPusherPressButton);
                robot.autoTimer.reset();

                if (robot.beaconColorSensor.alpha() > 5) {
                    colorCheck = robot.beaconColorSensor.red() - robot.beaconColorSensor.blue();

                    if (colorCheck > robot.colorVerified) {
                        robot.beaconPusher.setPosition(robot.beaconPusherRetract);
                        programState = State.TURN_TO_CAP_BALL;

                        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        rightTarget = robot.rightMotor.getCurrentPosition();
                        leftTarget = robot.leftMotor.getCurrentPosition() + (int)(135 * robot.countsPerTurnDegree());

                        robot.rightMotor.setTargetPosition(rightTarget);
                        robot.leftMotor.setTargetPosition(leftTarget);
                    }
                }
                break;

            case TURN_TO_CAP_BALL:
                robot.rightMotor.setPower(0);
                robot.leftMotor.setPower(1.0);

                if (Math.abs(robot.leftMotor.getCurrentPosition()- leftTarget) < 2) {
                    programState = State.FORWARD_TO_HIT_CAP_BALL;

                    rightTarget = robot.rightMotor.getCurrentPosition() + (int)(48 * robot.countsPerInch());
                    leftTarget = robot.leftMotor.getCurrentPosition() + (int)(48 * robot.countsPerInch());

                    robot.rightMotor.setTargetPosition(rightTarget);
                    robot.leftMotor.setTargetPosition(leftTarget);
                }
                break;

            case FORWARD_TO_HIT_CAP_BALL:
                robot.rightMotor.setPower(1.0);
                robot.leftMotor.setPower(1.0);

                if (Math.abs(robot.rightMotor.getCurrentPosition() - rightTarget) < 2  &&
                        Math.abs(robot.leftMotor.getCurrentPosition() - leftTarget) < 2) {
                    programState = State.END;
                }
                break;

            case END:
                robot.rightMotor.setPower(0);
                robot.leftMotor.setPower(0);
                break;

        }


    }

    @Override
    public void stop() {
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(0);
    }

}
