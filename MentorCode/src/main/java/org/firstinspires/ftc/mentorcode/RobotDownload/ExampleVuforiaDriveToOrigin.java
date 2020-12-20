/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.mentorcode.RobotDownload;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name = "Vuforia Drive to Origin", group = "Vuforia")
//@Disabled

public class ExampleVuforiaDriveToOrigin extends OpMode {

    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targetsUltimateGoal;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private VuforiaTrackable blueTowerGoalTarget;
    private VuforiaTrackable redTowerGoalTarget;
    private VuforiaTrackable redAllianceTarget;
    private VuforiaTrackable blueAllianceTarget;
    private VuforiaTrackable frontWallTarget;

    private OpenGLMatrix lastLocation = null;
    private OpenGLMatrix cameraLocation;
    private OpenGLMatrix robotLocationTransform;

    WebcamName webcamName = null;

    DcMotor frMotor, flMotor, brMotor, blMotor;
    BNO055IMU imu;
    Orientation gyroAngles;

    double driveDirection = 0, driveDirection360 = 0;   //Direction the robot should move
    double driveSpeed = 0;              //The speed the robot should move
    double driveRotation = 0;           //Causes the robot to rotate
    double desiredRobotHeading = 0;             //The direction the robot should be facing
    double flPower = 0, frPower = 0, blPower = 0, brPower = 0;  //calculated motor powers
    double maxMotorPower = 0;           //Variable used to figure out the max motor power so all motors can be scaled between 0 and 1
    float robotX, robotY, robotZ, robotAngle, x, y;
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    float mmFTCFieldWidth = (12 * 12) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    @Override
    public void init() {
        frMotor = hardwareMap.dcMotor.get("frontright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        brMotor = hardwareMap.dcMotor.get("backright");
        blMotor = hardwareMap.dcMotor.get("backleft");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        vuforiaInit();

        telemetry.addData("Status:", "Robot is Initialized");
    }


    @Override
    public void init_loop() {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }


    @Override
    public void start() {
        targetsUltimateGoal.activate();//Start tracking the data sets we care about.
    }


    @Override
    public void loop() {

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (robotLocationTransform != null) {

            VectorF trans = lastLocation.getTranslation();

            robotX = trans.get(0);
            robotY = trans.get(1);
            robotZ = trans.get(2);
            robotAngle = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;

            //Get the desired drive direction and speed from Vuforia
            //the values are scaled to meters.
            y = - robotY;
            x = - robotX;

            //finding the hypotenuse to calculate drive speed
            driveSpeed = Math.sqrt(y * y + x * x)/mmPerInch < 6 ? .25: .4;

            driveDirection = Math.atan2(-x,y);
            driveDirection360 = driveDirection >= 0 ? driveDirection : (2*Math.PI) + driveDirection;

            //Determine if the robot needs to rotate and if so in what direction.
            if (Math.abs(robotAngle - Math.toDegrees(desiredRobotHeading)) > 3) {
                driveRotation = (desiredRobotHeading - robotAngle) * .01;
            } else {
                driveRotation = 0;
            }

            //Calculate the power for each motor to go the direction, speed and rotation gathered above
            flPower = driveSpeed * Math.cos(driveDirection360 - Math.toRadians(robotAngle) + Math.PI / 4) - driveRotation;
            frPower = driveSpeed * Math.sin(driveDirection360 - Math.toRadians(robotAngle) + Math.PI / 4) + driveRotation;
            blPower = driveSpeed * Math.sin(driveDirection360 - Math.toRadians(robotAngle) + Math.PI / 4) - driveRotation;
            brPower = driveSpeed * Math.cos(driveDirection360 - Math.toRadians(robotAngle) + Math.PI / 4) + driveRotation;

            //Determine the maximum calulated motor power
            maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

            //If any motor power is not in the range {-1, 1}, scale all motors to fit in the range and still move as directed
            if (Math.abs(maxMotorPower) > 1) {
                flPower = flPower / maxMotorPower;
                frPower = frPower / maxMotorPower;
                blPower = blPower / maxMotorPower;
                brPower = brPower / maxMotorPower;
            } else if (Math.abs(maxMotorPower) < .1 || (Math.sqrt(y * y + x * x)/mmPerInch) < 1){
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }

            //Set the actual motor powers
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            blMotor.setPower(blPower);
            brMotor.setPower(brPower);
        }

            // send the info back to driver station using telemetry function.
            telemetry.addData("MotorLeft front", flPower);
            telemetry.addData("MotorLeft back", blPower);
            telemetry.addData("MotorRight front", frPower);
            telemetry.addData("MotorRight back", brPower);
            telemetry.addData("Drive Rotation", driveRotation);
            telemetry.addData("Robot Heading", robotAngle);
            telemetry.addData("Drive Speed", driveSpeed);
            telemetry.addData("Drive Direction", Math.toDegrees(driveDirection360));
            telemetry.addData("RobotX", robotX / mmPerInch);
            telemetry.addData("RobotY", robotY / mmPerInch);
            telemetry.addData("DriveX", x/ mmPerInch);
            telemetry.addData("DriveY", y/ mmPerInch);
            telemetry.addData("Pos", formatMatrix(lastLocation));

    }


    @Override
    public void stop() {
        targetsUltimateGoal.deactivate();
    }

    private void vuforiaInit() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam1");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.cameraDirection = FRONT;
        parameters.vuforiaLicenseKey = "Ab8M6H3/////AAABmdrDvsbcC0ksuYYrZ7fMQA5bzewUDais0KZfQ2jodz1/ImE2HslZR2fWU92Fyis+Ia/+Zz7LqR6lo1WBamZ/77uvaatxododdQnfyLwSw9TKmebFAuLltTmkG65ZlwGps/xyddg6XlwlddG9TbbSiG9poHchV+CVGHZ242eAyfvEEhhfS8p2KYlFj3QHZ9R5pXkJj2mlzksbu+FLgokG1fKPLgll+DaBi8U+nxS5rnSXHT3ZHIWN8au9BWQyUeP+qp9b0OLxD0fQP3XEbXTQuNJuAhrl0DxFpYs/uUPKWq9p7KyPtWiLVt6NDNfOvvA3lNQCJWsyF7rLWoZeWXBTBp2Ll4p/tqY3nAvxzGZz0l7Z";
        parameters.useExtendedTracking = false;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");

        // Setup the targets to be tracked
        blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        blueTowerGoalTarget.setLocation(createMatrix(0, 12*mmPerInch, 0, 90, 0 ,0));

        redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        redTowerGoalTarget.setLocation(createMatrix(0, 20*mmPerInch, 0, 90, 0 ,0));

        redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        redAllianceTarget.setLocation(createMatrix(0, 12*mmPerInch, 0, 90, 0 ,0));

        blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        blueAllianceTarget.setLocation(createMatrix(0, 12*mmPerInch, 0, 90, 0 ,0));

        frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
        frontWallTarget.setLocation(createMatrix(0, 12*mmPerInch, 0, 90, 0 ,0));

        allTrackables.addAll(targetsUltimateGoal);

        // Set phone location on robot
        cameraLocation = createMatrix(0, 0, 0, 90, 0, 180);

        //  Let all the trackable listeners know where the camera is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, cameraLocation);
        }

        lastLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }



    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        EXTRINSIC, XYZ, DEGREES, u, v, w));
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String formatMatrix(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}