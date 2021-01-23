package org.firstinspires.ftc.examplecode.Driving.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the robot has mechanum drive and uses the following device names:
 * Note:  All names are lower case.
 *
 * Motor channel 0:  Front Left drive motor:    "frontleft"
 * Motor channel 1:  Front Right drive motor:   "frontright"
 * Motor channel 2:  Back Left drive motor:     "backleft"
 * Motor channel 3:  Back Right drive motor:    "frontright"
 * I2C Bus 0:        Rev Hub IMU:               "imu"
 *
 */
public class OdometryBotHardware {
    /* Public OpMode members. */
    //Drive motors
    DcMotor left_front, right_front, left_back, right_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    //IMU Sensor
    BNO055IMU imu;
    //Hardware Map Names for drive motors and odometry wheels.
    String lfName = "frontleft", rfName = "frontright", lbName = "backleft", rbName = "backright";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 435.74278;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Constructor */
    public OdometryBotHardware() {
    }

    /************************************************************************************
     * Initialize standard Hardware interfaces
     */
    public void init(HardwareMap passhwMap) {
        // save reference to HW Map
        hwMap = passhwMap;

        //Initialize drive motors
        left_front = hwMap.dcMotor.get(lfName);
        right_front = hwMap.dcMotor.get(rfName);
        left_back = hwMap.dcMotor.get(lbName);
        right_back = hwMap.dcMotor.get(rbName);

        verticalLeft = hwMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hwMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hwMap.dcMotor.get(horizontalEncoderName);

        left_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.REVERSE);

        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize IMU parameters
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    /************************************************************************************
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    public double getZAngle(){
        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    /*************************************************************************************
     * Sets power to all four drive motors
     */
    public void setPowerAll(double lf, double rf, double lb, double rb){
        left_front.setPower(lf);
        right_front.setPower(rf);
        left_back.setPower(lb);
        right_back.setPower(rb);
    }
}
