package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

//IMU Imports


public class EVALibrary extends LinearOpMode {

    public DcMotor leftDrive;
    public DcMotor rightDrive;

    public RevTouchSensor liftButton;

    public DcMotor lndrUpDown;

    public DcMotor minSpin;
    public Servo mkrDeposit;


    //<editor-fold desc="Vuforia Variables">
// Setup variables
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
// Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // DogeCV detector
    GoldAlignDetector detector;
//</editor-fold>


    //IMU Declarations
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    int globalAngle;


    enum Rotation {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }


    public int getAngle() {
// We experimentally determined the Z axis is the axis we want to use for heading angle.
// We have to process the angle because the imu works in euler angles so the Z axis is
// returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
// 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        if (globalAngle <= -360)
            globalAngle = globalAngle + 360;
        else if (globalAngle >= 360)
            globalAngle = globalAngle - 360;
        return globalAngle;
    }

    public void init(HardwareMap hw) {
        leftDrive = hw.get(DcMotor.class, "leftDrive");
        rightDrive = hw.get(DcMotor.class, "rightDrive");
        liftButton = hw.get(RevTouchSensor.class, "liftArmButton");

        lndrUpDown = hw.get(DcMotor.class, "lndrUpDown");
        minSpin = hw.get(DcMotor.class, "minSpin");
        mkrDeposit = hw.get(Servo.class, "mkrDeposit");


        cameraInit(hw);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hw.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    @Override
    public void runOpMode() {

    }


    public void cameraInit(HardwareMap hw) {
        webcamName = hw.get(WebcamName.class, "Webcam 1");

// Set up parameters for Vuforia
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

// Vuforia licence key
        parameters.vuforiaLicenseKey = "Ac82VRT/////AAABmXFeOum/ykf5k4hgtJGJ5KshEp62RNkWxwSOwlZ3Kc6Rm99nR1D+0G1ucj+N3Yq29EAE2sutnuiR31pxYdi01PEa6iWMu0Q14XW5EKngMPlV9+tl86MDavCD/U5TJZ+Wt2VDTRH5ZA+JZSzRc2VadBSwgLTSilaqwPJIirBcM2HVdQ0uFJNY9UGpmXcWX3mT3CwcqybnEbljgjCmjd3g4NLcxSrjk/SWM/wJLTVTOkQRGzANwDTJPbW2aUId5PAVKSLUwPlQ+0/uzIM7xOt/Ne/ZSIzhU0L99CSqJcD0Hds4t8r4rrykO/KT7o9lJjOrrJxBiRMKmpfxmWiT7Y+JsB62iAyGFZtGxr/n0BmemOnk\n";
        parameters.fillCameraMonitorViewParent = true;

// Set camera name for Vuforia config
        parameters.cameraName = webcamName;

// Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

//Setup trackables
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

// For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

// Activate the targets
        targetsRoverRuckus.activate();

// Initialize the detector
        detector = new GoldAlignDetector();
        detector.init(hw.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;

// Set the detector
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
    }


    public void drive(double leftSpeed, double rightSpeed) {
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(-rightSpeed);
    }

    public void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }



    public void knockMinerals(Telemetry tw) {
        double speed = 0.17;
        if (detector.getAligned()) {
            return;

        }

        Rotation dir = Rotation.COUNTERCLOCKWISE;

        stopDrive();


        if (detector.getXPosition() < detector.getAlignXMin()) {
            while (!detector.getAligned() && !isStopRequested()) {
                drive(-speed, speed);
                tw.addData("x pos", detector.getXPosition());
                tw.update();
            }
        }
        if (detector.getXPosition() > detector.getAlignXMax()) {
            while (!detector.getAligned() && !isStopRequested()) {
                drive(speed, -speed);
                tw.addData("x pos", detector.getXPosition());
                tw.update();
            }
        }
        stopDrive();

    }

    public void turn(int aimDegrees, double speed, Rotation direction) {
        int currAngle = getAngle();
        while (!(currAngle < aimDegrees - 3) && !(currAngle < aimDegrees - 3)) {
            if (direction == Rotation.CLOCKWISE) {
                drive(speed, -speed);
            }
            if (direction == Rotation.COUNTERCLOCKWISE) {
                drive(-speed, speed);
            }
            currAngle = getAngle();
        }
        stopDrive();
    }

    public void turn(double speed, Rotation direction) {
        if (direction == Rotation.CLOCKWISE) {
            drive(speed, -speed);
        }
        if (direction == Rotation.COUNTERCLOCKWISE) {
            drive(-speed, speed);
        }
    }

    public boolean isLiftButtonUnpressed() {
        return !liftButton.isPressed();
    }



    public void landerMotor(double speed) {
        lndrUpDown.setPower(speed);
    }

    public void mineralSpinner(double speed) {
        minSpin.setPower(speed);
    }




}