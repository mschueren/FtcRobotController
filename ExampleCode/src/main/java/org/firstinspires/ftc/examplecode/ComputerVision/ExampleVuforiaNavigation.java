/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.examplecode.ComputerVision;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to drive to
 * a target.
 * The code is structured as a LinearOpMode
 * <p>
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 * <p>
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name = "Vuforia Read Target", group = "Vuforia")
@Disabled
public class ExampleVuforiaNavigation extends LinearOpMode {

    /**
     * {@link #vuforiaLocalizer} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
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

    float robotX, robotY, robotZ, robotAngle;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels



    @Override
    public void runOpMode() throws InterruptedException {

        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
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
        wheelsTarget.setLocation(createMatrix(0, 0, 0, 90, 0, 90));

        toolsTarget = velocityVortexImages.get(1);
        toolsTarget.setName("Tools");
        toolsTarget.setLocation(createMatrix(0, 0, 0, 90, 0 ,90));

        legosTarget = velocityVortexImages.get(2);
        legosTarget.setName("Legos");
        legosTarget.setLocation(createMatrix(0, 0, 0, 90, 0, 90));

        gearsTarget = velocityVortexImages.get(3);
        gearsTarget.setName("Gears");
        gearsTarget.setLocation(createMatrix(0, 0, 0, 90, 0, 90));


        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 90, 0, 0);

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


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        velocityVortexImages.activate();

        while (opModeIsActive()) {

            telemetry.addData(wheelsTarget.getName(), wheelsListener.isVisible() ? "Visible" : "Not Visible");
            telemetry.addData(toolsTarget.getName(), toolsListener.isVisible() ? "Visible" : "Not Visible");
            telemetry.addData(legosTarget.getName(), legosListener.isVisible() ? "Visible" : "Not Visible");
            telemetry.addData(gearsTarget.getName(), gearsListener.isVisible() ? "Visible" : "Not Visible");


            OpenGLMatrix robotLocationTransform = wheelsListener.getUpdatedRobotLocation();

            if (toolsListener.isVisible()) {
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

            }

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", formatMatrix(lastLocation));
                telemetry.addData("X Pos", robotX / mmPerInch);
                telemetry.addData("Y Pos", robotY / mmPerInch);
                telemetry.addData("Z Pos", robotZ / mmPerInch);
                telemetry.addData("Robot Angle", robotAngle);
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
            idle();
        }
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
