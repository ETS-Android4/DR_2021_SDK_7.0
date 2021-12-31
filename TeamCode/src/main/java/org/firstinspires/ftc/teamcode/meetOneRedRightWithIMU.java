package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


@Autonomous(name="meetOneRedRightWithIMU")
//@Disabled


public class meetOneRedRightWithIMU extends LinearOpMode
{

    public DcMotor motorRF;
    public DcMotor motorRB;
    public DcMotor motorLB;
    public DcMotor motorLF;

    private BNO055IMU imu;

    Orientation angles;


    @Override
    public void runOpMode() throws InterruptedException
    {

        int loopCount = 0;


        motorRF = hardwareMap.get(DcMotor.class,"motorRF");
        motorRB = hardwareMap.get(DcMotor.class,"motorRB");
        motorLB = hardwareMap.get(DcMotor.class,"motorLB");
        motorLF = hardwareMap.get(DcMotor.class,"motorLF");

        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse the appropriate motor

        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRB.setDirection(DcMotorSimple.Direction.REVERSE);



        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime whatever = new ElapsedTime();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //composeTelemetry();

        waitForStart();


        forward(500,.7);

        sleep();
        sleep();
        sleep();

        betterPivot(270);

        sleep();

        betterPivot(-270);

        sleep();
        sleep();


    }


    void forward(int distance, double power)
    {


        motorRF.setTargetPosition(distance + motorRF.getCurrentPosition());
        motorRB.setTargetPosition(distance + motorRB.getCurrentPosition());
        motorLF.setTargetPosition(distance + motorLF.getCurrentPosition());
        motorLB.setTargetPosition(distance + motorLB.getCurrentPosition());

        motorRF.setPower(power * .75);
        motorRB.setPower(power * .75);
        motorLB.setPower(power * .75);
        motorLF.setPower(power * .75);


        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("power" , power);
        telemetry.update();


    }

    void pivot(int distance, double power)
    {


        motorRF.setTargetPosition(distance + motorRF.getCurrentPosition());
        motorRB.setTargetPosition(distance + motorRB.getCurrentPosition());
        motorLF.setTargetPosition(-distance + motorLF.getCurrentPosition());
        motorLB.setTargetPosition(-distance + motorLB.getCurrentPosition());

        motorRF.setPower(power * .75);
        motorRB.setPower(power * .75);
        motorLB.setPower(power * -.75);
        motorLF.setPower(power * -.75);

        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    void betterPivot(int angle)
    {

        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double I = 0;
        double turnPower = 0;


        while (angle > 180)
        {
            angle -= 360;
        }
        while (angle < -180)
        {
            angle += 360;
        }

        while (I == 0)
        {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            while ( angles.firstAngle < angle + 2 && I == 0)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                turnPower = ((90 + angles.firstAngle) /90)*.75 + .15 ;

                motorRF.setPower(-turnPower);
                motorRB.setPower(-turnPower);
                motorLB.setPower(turnPower);
                motorLF.setPower(turnPower);

                telemetry.addData("right", I);
                telemetry.addData("current angle" , angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.update();
            }

            while(angles.firstAngle < angle - 3 && angles.firstAngle > angle + 3 && I == 0)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                motorRF.setPower(0);
                motorRB.setPower(0);
                motorLB.setPower(0);
                motorLF.setPower(0);

                I = 3;

                telemetry.addData("Dead", I);
                telemetry.addData("current angle" ,angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.update();
            }

            while(angles.firstAngle > angle - 2 && I == 0)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                turnPower = ((90 + angles.firstAngle) /90)*.75 + .15 ;

                motorRF.setPower(turnPower);
                motorRB.setPower(turnPower);
                motorLB.setPower(-turnPower);
                motorLF.setPower(-turnPower);

                telemetry.addData("Left", I);
                telemetry.addData("current angle" ,angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.update();
            }
            while(angles.firstAngle < angle - 3 && angles.firstAngle > angle + 3 && I == 0)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                motorRF.setPower(0);
                motorRB.setPower(0);
                motorLB.setPower(0);
                motorLF.setPower(0);

                I = 3;

                telemetry.addData("Dead", I);
                telemetry.addData("current angle" ,angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.update();
            }

        }


    }

    void sleep()
    {
        sleep(1000);
    }

    /*
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();

        }
        });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle); }
                });
    }
*/
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
//wobble.setTargetPosition(upPosition);
//                wobble.setPower(0.7);
//                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

