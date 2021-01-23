package org.firstinspires.ftc.examplecode.Driving.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.examplecode.Driving.Odometry.OdometryBotHardware;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {

    final double PIVOT_SPEED = 0.5;
    private double leftEncoderDirection = -1;
    private double rightEncoderDirection = 1;
    private double backEncoderDirection = 1;

    ElapsedTime timer = new ElapsedTime();

    OdometryBotHardware robot = new OdometryBotHardware();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        robot.init(hardwareMap);
        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(robot.getZAngle() < 90 && opModeIsActive()){

            if(robot.getZAngle() < 60) {
                robot.setPowerAll(-PIVOT_SPEED, PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED);
            }else{
                robot.setPowerAll(-PIVOT_SPEED/2, PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", robot.getZAngle());
            telemetry.update();
        }

        //Stop the robot
        robot.setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", robot.getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = robot.getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(robot.verticalLeft.getCurrentPosition()) + (Math.abs(robot.verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;


        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*robot.COUNTS_PER_INCH);

        horizontalTickOffset = robot.horizontal.getCurrentPosition()/Math.toRadians(robot.getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", robot.getZAngle());
            telemetry.addData("Vertical Left Position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

}
