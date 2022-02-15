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


@Autonomous(name="Gen2DuckBlue")
//@Disabled

public class Gen2DuckBlue extends LinearOpMode
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

    private OpenCvCamera webcam;//find webcam statement

    private static final int CAMERA_WIDTH  = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

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

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 200, 0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 100);

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
        
        
        //auto settings
        //parking path
        while(!gamepad1.a) {
            if (gamepad1.dpad_down && DpadDownToggle)
            {
                if(parkPos < 2) {
                    parkPos += 1;
                }

                DpadDownToggle = false;
            } else if (!gamepad1.dpad_down && !DpadDownToggle)
            {
                DpadDownToggle = true;
            }

            if (gamepad1.dpad_up && DpadUpToggle)
            {
                if(parkPos > 1) {
                    parkPos -= 1;
                }

                DpadUpToggle = false;
            } else if (!gamepad1.dpad_up && !DpadUpToggle)
            {
                DpadUpToggle = true;
            }

            if (parkPos == 1) {
                telemetry.addLine("depot");
            }

            else if (parkPos == 2) {
                telemetry.addLine("barrier");
            }

            else if (parkPos == 3) {
                telemetry.addLine("gap");
            }

            telemetry.update();
        }

        while(gamepad1.a) {}

        //TSE color 
        while(!gamepad1.a) {
            if (gamepad1.dpad_down && DpadDownToggle)
            {
                if(TSEColor < 9) {
                    TSEColor += 1;
                }

                DpadDownToggle = false;
            } else if (!gamepad1.dpad_down && !DpadDownToggle)
            {
                DpadDownToggle = true;
            }

            if (gamepad1.dpad_up && DpadUpToggle)
            {
                if(TSEColor > 1) {
                    TSEColor -= 1;
                }

                DpadUpToggle = false;
            } else if (!gamepad1.dpad_up && !DpadUpToggle)
            {
                DpadUpToggle = true;
            }



            if (TSEColor == 1) {
                telemetry.addLine("Red");

                scalarLowerYCrCb.val[0] = 16;//Y
                scalarLowerYCrCb.val[1] = 196;//Cr
                scalarLowerYCrCb.val[2] = 16;//Cb

                scalarUpperYCrCb.val[0] = 235;//Y
                scalarUpperYCrCb.val[1] = 240;//Cr
                scalarUpperYCrCb.val[2] = 150;//Cb
            }

            else if (TSEColor == 2) {
                telemetry.addLine("Orange");

                scalarLowerYCrCb.val[0] = 16;//Y
                scalarLowerYCrCb.val[1] = 195;//Cr
                scalarLowerYCrCb.val[2] = 0;//Cb

                scalarUpperYCrCb.val[0] = 235;//Y
                scalarUpperYCrCb.val[1] = 240;//Cr
                scalarUpperYCrCb.val[2] = 79;//Cb
            }

            else if (TSEColor == 3) {
                telemetry.addLine("Yellow");

                scalarLowerYCrCb.val[0] = 16;//Y
                scalarLowerYCrCb.val[1] = 140;//Cr
                scalarLowerYCrCb.val[2] = 16;//Cb

                scalarUpperYCrCb.val[0] = 235;//Y
                scalarUpperYCrCb.val[1] = 195;//Cr
                scalarUpperYCrCb.val[2] = 90;//Cb
            }

            else if (TSEColor == 4) {
                telemetry.addLine("Green");

                scalarLowerYCrCb.val[0] = 16;//Y
                scalarLowerYCrCb.val[1] = 16;//Cr
                scalarLowerYCrCb.val[2] = 16;//Cb

                scalarUpperYCrCb.val[0] = 235;//Y
                scalarUpperYCrCb.val[1] = 150;//Cr
                scalarUpperYCrCb.val[2] = 130;//Cb
            }

            else if (TSEColor == 5) {
                telemetry.addLine("Blue");

                scalarLowerYCrCb.val[0] = 16;//Y
                scalarLowerYCrCb.val[1] = 16;//Cr
                scalarLowerYCrCb.val[2] = 130;//Cb

                scalarUpperYCrCb.val[0] = 235;//Y
                scalarUpperYCrCb.val[1] = 130;//Cr
                scalarUpperYCrCb.val[2] = 240;//Cb
            }

            else if (TSEColor == 6) {
                telemetry.addLine("Purple");

                scalarLowerYCrCb.val[0] = 16;//Y
                scalarLowerYCrCb.val[1] = 120;//Cr
                scalarLowerYCrCb.val[2] = 120;//Cb

                scalarUpperYCrCb.val[0] = 235;//Y
                scalarUpperYCrCb.val[1] = 196;//Cr
                scalarUpperYCrCb.val[2] = 240;//Cb
            }

            else if (TSEColor == 7) {
                telemetry.addLine("Pink");

                scalarLowerYCrCb.val[0] = 16;//Y
                scalarLowerYCrCb.val[1] = 190;//Cr
                scalarLowerYCrCb.val[2] = 120;//Cb

                scalarUpperYCrCb.val[0] = 235;//Y
                scalarUpperYCrCb.val[1] = 240;//Cr
                scalarUpperYCrCb.val[2] = 240;//Cb
            }

            else if (TSEColor == 8) {
                telemetry.addLine("White");

                scalarLowerYCrCb.val[0] = 16;//Y
                scalarLowerYCrCb.val[1] = 16;//Cr
                scalarLowerYCrCb.val[2] = 16;//Cb

                scalarUpperYCrCb.val[0] = 125;//Y
                scalarUpperYCrCb.val[1] = 240;//Cr
                scalarUpperYCrCb.val[2] = 240;//Cb
            }

            else if (TSEColor == 9) {
                telemetry.addLine("Black");

                scalarLowerYCrCb.val[0] = 126;//Y
                scalarLowerYCrCb.val[1] = 16;//Cr
                scalarLowerYCrCb.val[2] = 16;//Cb

                scalarUpperYCrCb.val[0] = 235;//Y
                scalarUpperYCrCb.val[1] = 240;//Cr
                scalarUpperYCrCb.val[2] = 240;//Cb
            }

            telemetry.update();
        }


        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline());
        // Configuration of Pipeline
        myPipeline.ConfigurePipeline(0, 0,50,85,  CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.update();

        while (!isStarted())
        {
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
            // testing(myPipeline);

            // Watch our YouTube Tutorial for the better explanation

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if((myPipeline.getRectMidpointX() - 160) > 50){
                    AUTONOMOUS_C();
                    barPos = 1;
                }
                else if((myPipeline.getRectMidpointX() - 160) < -50){
                    AUTONOMOUS_A();
                    barPos = 3;
                }
                else {
                    AUTONOMOUS_B();
                    barPos = 2;
                }
            }

            clawL.setPosition(.26);
        }

        waitForStart();

        webcam.stopStreaming();

        drive(0, -1, 0, 1, 500);
        
        sleep(250);
        
        betterPivot(-90);
        
        sleep(750);
        
        drive(90, 1, 0, .5, 2000);
        
        sleep(250);
        
        duckSpinnerLeft.setPower(-.5);
        duckSpinnerRight.setPower(-.5);
        
        drive(90, .5, .5, .25, 1750);
        
        sleep(4500);
        
        duckSpinnerLeft.setPower(0);
        duckSpinnerRight.setPower(0);
        
        basicEncoderDrive(90, 0, -1, 1, -1500);

        sleep(250);

        if(barPos == 1) {
            clawL.setPosition(.26);

            sleep(150);

            armL.setPosition(.5);
            slidesL.setPosition(.45);
        }

        else if(barPos == 2) {
            clawL.setPosition(.26);

            sleep(150);

            armL.setPosition(.55);
            slidesL.setPosition(0);
        }

        else if(barPos == 3) {
            clawL.setPosition(.26);

            sleep(150);

            armL.setPosition(.6);
            slidesL.setPosition(0);
        }

        sleep(250);

        basicSensorDrive(90, -.75, 0, 1, 20);

        basicEncoderDrive(90, -.75, 0, 1, 750);
        
        sleep(250);
        
        clawL.setPosition(0);
        
        sleep(1000);
        
        drive(90, 1, 0, .5, 2000);
        
        armL.setPosition(0);
        slidesL.setPosition(0);
        
        sleep(250);
        
        basicEncoderDrive(90, 0, 1, 1, 300);

        sleep(1000);

    }


    public void betterPivot(int angle)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);


        double I = 0;
        double turnPower;


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

            while ( angles.firstAngle < angle + 10 && I == 0)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                turnPower = ((angle - angles.firstAngle) /angle)+ .2;

                robot.motorRF.setPower(turnPower);
                robot.motorRB.setPower(turnPower);
                robot.motorLB.setPower(-turnPower);
                robot.motorLF.setPower(-turnPower);

                telemetry.addData("Left", I);
                telemetry.addData("current angle" , angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.update();

                if(angles.firstAngle > angle - 2 && I == 0)
                {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    robot.motorRF.setPower(0);
                    robot.motorRB.setPower(0);
                    robot.motorLB.setPower(0);
                    robot.motorLF.setPower(0);

                    I = 3;

                    telemetry.addData("Dead", I);
                    telemetry.addData("current angle" ,angles.firstAngle);
                    telemetry.addData("target angle" , angle);
                    telemetry.update();
                }
            }

            while(angles.firstAngle > angle -  10 && I == 0)
            {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                turnPower = ((angle - angles.firstAngle) /angle)+ .2;

                robot.motorRF.setPower(-turnPower);
                robot.motorRB.setPower(-turnPower);
                robot.motorLB.setPower(turnPower);
                robot.motorLF.setPower(turnPower);

                telemetry.addData("right", I);
                telemetry.addData("current angle" ,angles.firstAngle);
                telemetry.addData("target angle" , angle);
                telemetry.update();

                if(angles.firstAngle < angle + 2 && I == 0)
                {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    robot.motorRF.setPower(0);
                    robot.motorRB.setPower(0);
                    robot.motorLB.setPower(0);
                    robot.motorLF.setPower(0);

                    I = 3;

                    telemetry.addData("Dead", I);
                    telemetry.addData("current angle" ,angles.firstAngle);
                    telemetry.addData("target angle" , angle);
                    telemetry.update();
                }
            }


        }


    }

    public void  betterTimeDrive(int angle, double PowerX, double PowerY, double speed, double time)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        ElapsedTime driveTime = new ElapsedTime();

        driveTimeVar = driveTime.milliseconds();

        while ((driveTimeVar + time) > driveTime.milliseconds())
        {

            double turnPower;

            while (angle > 180)
            {
                angle -= 360;
            }
            while (angle < -180)
            {
                angle += 360;
            }


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPower = ((angle - angles.firstAngle) / angle) + .2;

            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (turnPower)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (turnPower));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.update();
        }

        stop(1);
    }
    
    
    public void  betterEncoderDrive(int angle, double PowerX, double PowerY, double speed, double distance)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        
        encoderReadingRF = robot.motorRF.getCurrentPosition();
        target = (encoderReadingRF + distance);
        
        if(distance < 0) {

        while (robot.motorRF.getCurrentPosition() <= target)
        {

            double turnPower;

            while (angle > 180)
            {
                angle -= 360;
            }
            while (angle < -180)
            {
                angle += 360;
            }


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPower = ((angle - angles.firstAngle) / angle) + .2;

            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (turnPower)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (turnPower));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.update();
        }
        
        }
        
        else if(distance > 0) {

        while (robot.motorRF.getCurrentPosition() >= target)
        {

            double turnPower;

            while (angle > 180)
            {
                angle -= 360;
            }
            while (angle < -180)
            {
                angle += 360;
            }


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPower = ((angle - angles.firstAngle) / angle) + .2;

            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (turnPower)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (turnPower));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.update();
        }
        
        }

        stop(1);
    }
    
    
    
    public void  betterSensorDrive(int angle, double PowerX, double PowerY, double speed, double distance)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        
        DistanceSensor sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        
        if(distance < sensorRange.getDistance(DistanceUnit.INCH)) {

        while (distance <= sensorRange.getDistance(DistanceUnit.INCH))
        {

            double turnPower;

            while (angle > 180)
            {
                angle -= 360;
            }
            while (angle < -180)
            {
                angle += 360;
            }


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPower = ((angle - angles.firstAngle) / angle) + .2;

            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (turnPower)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (turnPower));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.update();
        }
        
        }
        
        else if(distance > sensorRange.getDistance(DistanceUnit.INCH)) {

        while (distance >= sensorRange.getDistance(DistanceUnit.INCH))
        {

            double turnPower;

            while (angle > 180)
            {
                angle -= 360;
            }
            while (angle < -180)
            {
                angle += 360;
            }


            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            turnPower = ((angle - angles.firstAngle) / angle) + .2;

            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (turnPower)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (turnPower)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (turnPower));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.update();
        }
        
        }

        stop(1);
    }


    public void  basicEncoderDrive(int angle, double PowerX, double PowerY, double speed, double distance)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        encoderReadingRF = robot.motorRF.getCurrentPosition();
        target = (encoderReadingRF + distance);

        if(distance > 0) {

            while (robot.motorRF.getCurrentPosition() <= target)
            {



                robot.motorRF.setPower(speed * ((PowerY - PowerX) - (0)));
                robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (0)));
                robot.motorLB.setPower(speed * ((PowerY - PowerX) - (0)));
                robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (0));

                telemetry.addData("motorRF Power", robot.motorRF.getPower());
                telemetry.addData("motorRB Power", robot.motorRB.getPower());
                telemetry.addData("motorLB Power", robot.motorLB.getPower());
                telemetry.addData("motorLF Power", robot.motorLF.getPower());
                telemetry.update();
            }

        }

        else if(distance < 0) {

            while (robot.motorRF.getCurrentPosition() >= target)
            {

                robot.motorRF.setPower(speed * ((PowerY - PowerX) - (0)));
                robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (0)));
                robot.motorLB.setPower(speed * ((PowerY - PowerX) - (0)));
                robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (0));

                telemetry.addData("motorRF Power", robot.motorRF.getPower());
                telemetry.addData("motorRB Power", robot.motorRB.getPower());
                telemetry.addData("motorLB Power", robot.motorLB.getPower());
                telemetry.addData("motorLF Power", robot.motorLF.getPower());
                telemetry.update();
            }

        }

        stop(1);
    }


    public void  basicSensorDrive(int angle, double PowerX, double PowerY, double speed, double distance)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        DistanceSensor sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        if(distance < sensorRange.getDistance(DistanceUnit.INCH)) {

            while (distance <= sensorRange.getDistance(DistanceUnit.INCH))
            {



                robot.motorRF.setPower(speed * ((PowerY - PowerX) - (0)));
                robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (0)));
                robot.motorLB.setPower(speed * ((PowerY - PowerX) - (0)));
                robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (0));

                telemetry.addData("motorRF Power", robot.motorRF.getPower());
                telemetry.addData("motorRB Power", robot.motorRB.getPower());
                telemetry.addData("motorLB Power", robot.motorLB.getPower());
                telemetry.addData("motorLF Power", robot.motorLF.getPower());
                telemetry.addData("distance", sensorRange.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

        }

        else if(distance > sensorRange.getDistance(DistanceUnit.INCH)) {

            while (distance >= sensorRange.getDistance(DistanceUnit.INCH))
            {



                robot.motorRF.setPower(speed * ((PowerY - PowerX) - (0)));
                robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (0)));
                robot.motorLB.setPower(speed * ((PowerY - PowerX) - (0)));
                robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (0));

                telemetry.addData("motorRF Power", robot.motorRF.getPower());
                telemetry.addData("motorRB Power", robot.motorRB.getPower());
                telemetry.addData("motorLB Power", robot.motorLB.getPower());
                telemetry.addData("motorLF Power", robot.motorLF.getPower());
                telemetry.addData("distance", sensorRange.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

        }

        stop(1);
    }



    public void  drive(int angle, double PowerX, double PowerY, double speed, double time)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        ElapsedTime driveTime = new ElapsedTime();

        driveTimeVar = driveTime.milliseconds();

        while ((driveTimeVar + time) > driveTime.milliseconds())
        {
            robot.motorRF.setPower(speed * ((PowerY - PowerX) - (0)));
            robot.motorRB.setPower(speed * (-(-PowerX - PowerY) - (0)));
            robot.motorLB.setPower(speed * ((PowerY - PowerX) - (0)));
            robot.motorLF.setPower(speed * ((PowerX + PowerY)) - (0));

            telemetry.addData("motorRF Power", robot.motorRF.getPower());
            telemetry.addData("motorRB Power", robot.motorRB.getPower());
            telemetry.addData("motorLB Power", robot.motorLB.getPower());
            telemetry.addData("motorLF Power", robot.motorLF.getPower());
            telemetry.addData("time", driveTime.milliseconds());
            telemetry.update();
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


    /*
    void forward (int distance, double power) {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setTargetPosition(distance + robot.motorRF.getCurrentPosition());
        robot.motorRB.setTargetPosition(distance + robot.motorRB.getCurrentPosition());
        robot.motorLM.setTargetPosition(distance + robot.motorLM.getCurrentPosition());
        robot.motorLB.setTargetPosition(distance + robot.motorLB.getCurrentPosition());

        robot.motorRF.setPower(power * .75);
        robot.motorRB.setPower(power * .75);
        robot.motorLB.setPower(power * .75);
        robot.motorLF.setPower(power * .75);

        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


     */

    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");
    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");
    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
    }
}
