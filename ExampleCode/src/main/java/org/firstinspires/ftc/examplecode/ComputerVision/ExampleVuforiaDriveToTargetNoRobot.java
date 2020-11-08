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

package org.firstinspires.ftc.examplecode.ComputerVision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

@Autonomous(name = "Vuforia Drive to Target No Bot", group = "Vuforia")
//@Disabled

public class ExampleVuforiaDriveToTargetNoRobot extends OpMode {
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaTrackables velocityVortexImages;
    private VuforiaTrackable wheelsTarget;
    private VuforiaTrackable toolsTarget;
    private VuforiaTrackable legosTarget;
    private VuforiaTrackable gearsTarget;
    private VuforiaTrackableDefaultListener wheelsListener;
    private VuforiaTrackableDefaultListener toolsListener;
    private VuforiaTrackableDefaultListener legosListener;
    private VuforiaTrackableDefaultListener gearsListener;

    public static final String TAG = "Vuforia Sample";

    private OpenGLMatrix lastLocation;
    private OpenGLMatrix phoneLocation;
    private OpenGLMatrix robotLocationTransform;

    double driveDirection = 0;          //Direction the robot should move
    double driveSpeed = 0;              //The speed the robot should move
    double driveRotation = 0;           //Causes the robot to rotate
    double desiredRobotHeading = 0;             //The direction the robot should be facing
    double flPower = 0, frPower = 0, blPower = 0, brPower = 0;  //calculated motor powers
    double maxMotorPower = 0;           //Variable used to figure out the max motor power so all motors can be scaled between 0 and 1
    float robotX, robotY, robotZ, robotAngle;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    @Override
    public void init() {
        vuforiaInit();
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        velocityVortexImages.activate();//Start tracking the data sets we care about.
        robotLocationTransform = wheelsListener.getUpdatedRobotLocation();
    }


    @Override
    public void loop() {

        if (wheelsListener.isVisible()) {
            robotLocationTransform = wheelsListener.getUpdatedRobotLocation();
        } else if (toolsListener.isVisible()) {
            robotLocationTransform = toolsListener.getUpdatedRobotLocation();
        } else if (legosListener.isVisible()) {
            robotLocationTransform = legosListener.getUpdatedRobotLocation();
        } else if (gearsListener.isVisible()) {
            robotLocationTransform = gearsListener.getUpdatedRobotLocation();
        }

        if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;

            VectorF trans = robotLocationTransform.getTranslation();

            robotX = trans.get(0);
            robotY = trans.get(1);
            robotZ = trans.get(2);
            robotAngle = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            //Get the desired drive direction and speed from Vuforia
            //the values are scaled to meters.
            float y = -robotY;
            float x = -robotX;

            //finding the hypotenuse to calculate drive speed
            driveSpeed = Math.min(0.5, Math.sqrt(y * y + x * x));

            //Find the direction to the target in radians.  Java uses radians in its trig functions
            if (x < 0 && y > 0) {
                desiredRobotHeading = Math.atan(-x / y);
            } else if (x < 0 && y == 0) {
                desiredRobotHeading = Math.PI / 2;
            } else if (x < 0 && y < 0) {
                desiredRobotHeading = Math.atan(-y / -x) + (Math.PI / 2);
            } else if (x == 0 && y < 0) {
                desiredRobotHeading = Math.PI;
            } else if (x > 0 && y < 0) {
                desiredRobotHeading = Math.atan(x / -y) + Math.PI;
            } else if (x > 0 && y == 0) {
                desiredRobotHeading = Math.PI * 3 / 2;
            } else if (x > 0 && y > 0) {
                desiredRobotHeading = Math.atan(y / x) + (Math.PI * 3 / 2);
            } else {
                desiredRobotHeading = 0;
            }

            driveDirection = desiredRobotHeading + (Math.toRadians(robotAngle) - Math.PI / 2);

            //Determine if the robot needs to rotate and if so in what direction.
            if (Math.abs(robotAngle - Math.toDegrees(desiredRobotHeading)) > 3) {
                driveRotation = (desiredRobotHeading - robotAngle) * .02;
            } else {
                driveRotation = 0;
            }

            //Calculate the power for each motor to go the direction, speed and rotation gathered above
            flPower = driveSpeed * Math.cos(driveDirection - Math.toRadians(robotAngle) + Math.PI / 4) - driveRotation;
            frPower = driveSpeed * Math.sin(driveDirection - Math.toRadians(robotAngle) + Math.PI / 4) + driveRotation;
            blPower = driveSpeed * Math.sin(driveDirection - Math.toRadians(robotAngle) + Math.PI / 4) - driveRotation;
            brPower = driveSpeed * Math.cos(driveDirection - Math.toRadians(robotAngle) + Math.PI / 4) + driveRotation;

            //Determine the maximum calulated motor power
            maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));

            //If any motor power is not in the range {-1, 1}, scale all motors to fit in the range and still move as directed
            if (Math.abs(maxMotorPower) > 1) {
                flPower = flPower / maxMotorPower;
                frPower = frPower / maxMotorPower;
                blPower = blPower / maxMotorPower;
                brPower = brPower / maxMotorPower;
            } else if (Math.abs(maxMotorPower) < .1) {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }
        }

        // send the info back to driver station using telemetry function.
        telemetry.addData("MotorLeft front", flPower);
        telemetry.addData("MotorLeft back", blPower);
        telemetry.addData("MotorRight front", frPower);
        telemetry.addData("MotorRight back", brPower);
        telemetry.addData("Drive Rotation", driveRotation);
        telemetry.addData("Robot Heading", robotAngle);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Drive Direction", Math.toDegrees(driveDirection));
        telemetry.addData("RobotX", robotX);
        telemetry.addData("RobotY", robotY);
        telemetry.addData(wheelsTarget.getName(), wheelsListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData(toolsTarget.getName(), toolsListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData(legosTarget.getName(), legosListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData(gearsTarget.getName(), gearsListener.isVisible() ? "Visible" : "Not Visible");
        telemetry.addData("Pos", formatMatrix(lastLocation));
    }


    @Override
    public void stop() {

    }

    private void vuforiaInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Ab8M6H3/////AAABmdrDvsbcC0ksuYYrZ7fMQA5bzewUDais0KZfQ2jodz1/ImE2HslZR2fWU92Fyis+Ia/+Zz7LqR6lo1WBamZ/77uvaatxododdQnfyLwSw9TKmebFAuLltTmkG65ZlwGps/xyddg6XlwlddG9TbbSiG9poHchV+CVGHZ242eAyfvEEhhfS8p2KYlFj3QHZ9R5pXkJj2mlzksbu+FLgokG1fKPLgll+DaBi8U+nxS5rnSXHT3ZHIWN8au9BWQyUeP+qp9b0OLxD0fQP3XEbXTQuNJuAhrl0DxFpYs/uUPKWq9p7KyPtWiLVt6NDNfOvvA3lNQCJWsyF7rLWoZeWXBTBp2Ll4p/tqY3nAvxzGZz0l7Z";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        velocityVortexImages = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4); //Will track all 4 images

        // Setup the targets to be tracked
        wheelsTarget = velocityVortexImages.get(0);
        wheelsTarget.setName("Wheels");
        wheelsTarget.setLocation(createMatrix(-150, 0, 0, 90, 0 ,90));

        toolsTarget = velocityVortexImages.get(1);
        toolsTarget.setName("Tools");
        toolsTarget.setLocation(createMatrix(-150, 0, 0, 90, 0 ,90));

        legosTarget = velocityVortexImages.get(2);
        legosTarget.setName("Legos");
        legosTarget.setLocation(createMatrix(-150, 0, 0, 90, 0 ,90));

        gearsTarget = velocityVortexImages.get(3);
        gearsTarget.setName("Gears");
        gearsTarget.setLocation(createMatrix(-150, 0, 0, 90, 0, 90));


        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 90, 0, 180);

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        wheelsListener = (VuforiaTrackableDefaultListener) wheelsTarget.getListener();
        wheelsListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        toolsListener = (VuforiaTrackableDefaultListener) toolsTarget.getListener();
        toolsListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        legosListener = (VuforiaTrackableDefaultListener) legosTarget.getListener();
        legosListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
        gearsListener = (VuforiaTrackableDefaultListener) gearsTarget.getListener();
        gearsListener.setPhoneInformation(phoneLocation, parameters.cameraDirection);


        lastLocation = createMatrix(0, 0, 0, 0, 0, 0);
    }



    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String formatMatrix(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}