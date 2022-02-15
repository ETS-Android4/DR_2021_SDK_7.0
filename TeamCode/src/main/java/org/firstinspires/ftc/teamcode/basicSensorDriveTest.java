package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pinkCode.ContourPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="basicSensorDriveTest")
//@Disabled

public class basicSensorDriveTest extends LinearOpMode
{
    BNO055IMU imu;

    public DcMotor intake1 = null;
    public DcMotor intake2 = null;
    public DcMotor duckSpinnerLeft = null;
    public  DcMotor duckSpinnerRight = null;
    public Servo clawL = null;
    public Servo  armL = null;
    public Servo  slidesL = null;
    
    Orientation angles;

    double barPos = 3;
    double driveTimeVar;
    boolean DpadUpToggle = true;
    boolean DpadDownToggle = true;
    int parkPos = 1;
    int TSEColor = 1;



    double CrLowerUpdate = 150;
    double CbLowerUpdate = 120;
    double CrUpperUpdate = 255;
    double CbUpperUpdate = 255;

    double lowerruntime = 0;
    double upperruntime = 0;

    double encoderReadingRF = 0;
    double target = 0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        duckSpinnerLeft = hardwareMap.dcMotor.get("duckSpinnerLeft");
        duckSpinnerRight = hardwareMap.dcMotor.get("duckSpinnerRight");
        clawL = hardwareMap.servo.get("clawL");
        armL = hardwareMap.servo.get("armL");
        slidesL = hardwareMap.servo.get("slidesL");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        

        waitForStart();

        basicSensorDrive(0, -.1, 0, 1, 30);
        
        basicSensorDrive(0, .1, 0, 1, 15);

    }


  
    
    
    public void  basicSensorDrive(int angle, double PowerX, double PowerY, double speed, double distance)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        
        DistanceSensor sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        
        double range = sensorRange.getDistance(DistanceUnit.INCH);
        
        if(distance < range) {

        while (distance <= range)
        {
            if(sensorRange.getDistance(DistanceUnit.INCH) < 35)
            {
                range = sensorRange.getDistance(DistanceUnit.INCH);
            }
            

            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (0)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (0)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (0)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (0));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.addData("distance", sensorRange.getDistance(DistanceUnit.INCH));
            telemetry.addData("range", range);
            telemetry.update();
        }
        
        }
        
        else if(distance > range) {

        while (distance >= range)
        {
        
            if(sensorRange.getDistance(DistanceUnit.INCH) < 35)
            {
                range = sensorRange.getDistance(DistanceUnit.INCH);
            }

            

            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (0)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (0)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (0)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (0));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.addData("distance", sensorRange.getDistance(DistanceUnit.INCH));
            telemetry.addData("range", range);
            telemetry.update();
        }
        
        }

        stop(1);
    }



    
    public void stop(int hi)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
    }


    
}
