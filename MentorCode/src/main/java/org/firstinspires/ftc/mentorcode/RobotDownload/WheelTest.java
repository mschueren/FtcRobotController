/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.mentorcode.RobotDownload;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 */

@TeleOp(name="Wheel Test", group="Example")
//@Disabled
public class WheelTest extends OpMode
{
    // Declare OpMode members.
    private DcMotor frMotor, flMotor, brMotor, blMotor;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        frMotor = hardwareMap.dcMotor.get("frontright");
        flMotor = hardwareMap.dcMotor.get("frontleft");
        brMotor = hardwareMap.dcMotor.get("backright");
        blMotor = hardwareMap.dcMotor.get("backleft");

        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() { }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(gamepad1.y){
            frMotor.setPower(.3);
        } else if(gamepad1.x){
            flMotor.setPower(.3);
        } else if(gamepad1.b){
            brMotor.setPower(.3);
        } else if(gamepad1.a){
            blMotor.setPower(.3);
        } else if(gamepad1.dpad_up){
            frMotor.setPower(.3);
            flMotor.setPower(.3);
            brMotor.setPower(.3);
            blMotor.setPower(.3);
        } else if(gamepad1.dpad_down){
            frMotor.setPower(-.3);
            flMotor.setPower(-.3);
            brMotor.setPower(-.3);
            blMotor.setPower(-.3);
        } else if(gamepad1.dpad_right){
            frMotor.setPower(-.3);
            flMotor.setPower(.3);
            brMotor.setPower(-.3);
            blMotor.setPower(.3);
        } else if(gamepad1.dpad_left) {
            frMotor.setPower(.3);
            flMotor.setPower(-.3);
            brMotor.setPower(.3);
            blMotor.setPower(-.3);
        } else if(gamepad1.right_trigger > .1){
            frMotor.setPower(0);
            flMotor.setPower(gamepad1.right_trigger);
            brMotor.setPower(gamepad1.right_trigger);
            blMotor.setPower(0);
        } else if(gamepad1.left_trigger > .1) {
            frMotor.setPower(gamepad1.left_trigger);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(gamepad1.left_trigger);
        } else {
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
        }

        String rightDirections = "Front Motor = " + frMotor.getDirection().toString() +
                                    " - Back Motor = " + brMotor.getDirection().toString();
        // Show the wheel power.
        telemetry.addData("Motor FR (y)", frMotor.getPower());
        telemetry.addData("Motor BR (b)", brMotor.getPower());
        telemetry.addData("Motor FL (x)", flMotor.getPower());
        telemetry.addData("Motor BL (a)", blMotor.getPower());
        telemetry.addData("Move Forward (Dpad Up)", "");
        telemetry.addData("Move Backwards (Dpad Down)", "");
        telemetry.addData("Rotate Clockwise (Dpad Right)", "");
        telemetry.addData("Rotate Counter-Clockwise (Dpad Left)", "");
        telemetry.addData("Move Diagonal Forward Right (Right Trigger)", "");
        telemetry.addData("Move Diagonal Forward Left (Left Trigger)", "");
        telemetry.addData("Front Right Motor Direction", frMotor.getDirection().toString());
        telemetry.addData("Back Right Motor Direction", brMotor.getDirection().toString());
        telemetry.addData("Front Left Motor Direction", flMotor.getDirection().toString());
        telemetry.addData("Back Left Motor Direction", blMotor.getDirection().toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
