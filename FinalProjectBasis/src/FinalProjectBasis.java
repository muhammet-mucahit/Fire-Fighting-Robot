import com.ridgesoft.intellibrain.IntelliBrain;
import com.ridgesoft.intellibrain.IntelliBrainDigitalIO;
import com.ridgesoft.io.Display;
import com.ridgesoft.robotics.AnalogInput;
import com.ridgesoft.robotics.ContinuousRotationServo;
import com.ridgesoft.robotics.Motor;
import com.ridgesoft.robotics.RangeFinder;
import com.ridgesoft.robotics.sensors.ParallaxPing;

public class FinalProjectBasis {

    //****** CONSTANTS ******
    // Possible states of the finite state machine
    private static final int WAIT = 0;
    private static final int NAV_RIGHT = 1;
    private static final int CENTER = 2;
    private static final int PUT_OUT = 3;

    private static final int LEFT_BUMPER = 100;
    private static final int RIGHT_BUMPER = 101;

    private static final int FLAME_PRESENT_LIMIT = 800;
    private static final int FLAME_NEAR_LIMIT = 900;

    // Possible floor Tags
    private static final int LINE_TAG = 0;
    private static final int CIRCLE_TAG = 1;
    private static final int NO_TAG = 2;

    // Algorithm Parameters
    private static final int DIST_TO_SIDE_WALL = 15;
    private static final int DIST_TO_FRONT_WALL = 15;
    private static final int DIST_TO_CANDLE = 20;
    private static final int DELTA_LIMIT = 4;
    private static final float GAIN = 0.9f;
    private static final float TURN_FACTOR = 7.5f; //8 ONCEKI DEGERLERE BAK BIZ NE YAPMISIZ!
    private static final int GO_VELO = 8;
    private static final int TURN_VELO = 5;
    private static final int LINE_LIMIT = 80;

    //****** HARDWARE OBJECTS AND VARIABLES ******
    private static int state;

    private static IntelliBrainDigitalIO startButton;
    private static IntelliBrainDigitalIO stopButton;

    private static IntelliBrainDigitalIO lBumber;
    private static IntelliBrainDigitalIO rBumber;

    private static RangeFinder lSonar;
    private static RangeFinder fSonar;
    private static RangeFinder rSonar;
    private static int lDist;
    private static int fDist;
    private static int rDist;

    private static Motor lMotor;
    private static Motor rMotor;

    private static Display lcd;

    private static AnalogInput lineSensor;
    private static int totalLines;

    private static FlameSensor flameSensor;
    private static IntelliBrainDigitalIO flameLED;

    private static Motor fan;

    public static void main(String[] args) {
        //****** Construction of the hardware objects ******
        startButton = IntelliBrain.getDigitalIO(12);
        startButton.setPullUp(true);
        stopButton = IntelliBrain.getDigitalIO(11);
        stopButton.setPullUp(true);

        lBumber = IntelliBrain.getDigitalIO(1);
        lBumber.setPullUp(true);
        rBumber = IntelliBrain.getDigitalIO(10);
        rBumber.setPullUp(true);

        lcd = IntelliBrain.getLcdDisplay();

        lineSensor = IntelliBrain.getAnalogInput(7);
        totalLines = 0;

        flameSensor = new FlameSensor(FLAME_PRESENT_LIMIT, FLAME_NEAR_LIMIT);

        flameLED = IntelliBrain.getDigitalIO(13);

        fan = IntelliBrain.getMotor(2);

        lSonar = new ParallaxPing(IntelliBrain.getDigitalIO(3));
        fSonar = new ParallaxPing(IntelliBrain.getDigitalIO(4));
        rSonar = new ParallaxPing(IntelliBrain.getDigitalIO(5));

        lMotor = new ContinuousRotationServo(IntelliBrain.getServo(1), false, 14);
        rMotor = new ContinuousRotationServo(IntelliBrain.getServo(2), true, 14);

        //****** State Machine ******
        state = WAIT;

        while(true) {
            switch (state) {
                case WAIT:
                    state = waitState();
                    break;
                case NAV_RIGHT:
                    state = navRigthState();
                    break;
                case CENTER:
                     state = centerState();
                    break;
                case PUT_OUT:
                     state = putOutState();
                    break;
            }

            displayState();
        }
    }

    private static int waitState() {
        while (startButton.isSet()) {
            //lcd.print(1, "Line: " + lineSensor.sample());
            flameSensor.scan();
            lcd.print(0, "Value: " + flameSensor.flameVal);
            lcd.print(1, "Direction: " + flameSensor.flameDir);
        }
        return NAV_RIGHT;
    }

    private static int navRigthState() {
        // Algorithm of the state
        rDist = (int) getDist(rSonar);
        fDist = (int) getDist(fSonar);

        if (fDist < DIST_TO_FRONT_WALL)
            rotateAngle(90);
        else if (!lBumber.isSet())
            bumperManuever(LEFT_BUMPER);
        else if (!rBumber.isSet())
            bumperManuever(RIGHT_BUMPER);

        int error = rDist - DIST_TO_SIDE_WALL;
        int delta = (int) (error * GAIN);

        if (delta > DELTA_LIMIT)      delta = DELTA_LIMIT;
        else if(delta < -DELTA_LIMIT) delta = -DELTA_LIMIT;

        move(GO_VELO, delta);

        if (getFloorTag() == LINE_TAG) {
            totalLines++;
            lcd.print(1, "Lines: " + totalLines);
        }

        // Conditions
         if(flameSensor.scan() > 0) {
            flameLED.set();
            return CENTER;
         }

         return NAV_RIGHT;
    }

    private static int centerState() {

        switch (flameSensor.scan()) {
            case 1:
                rotate(-3);
                break;
            case 2:
                rotate(-3);
                break;
            case 3:
                // It is alternative way to stop in front of the candle. First way.
                // By calculating distance with sonar sensor
                fDist = (int) getDist(fSonar);
                move(GO_VELO / 2, 0);
                if (fDist < DIST_TO_CANDLE) {
                    stop();
                    return PUT_OUT;
                }
                break;
            case 4:
                rotate(3);
                break;
            case 5:
                rotate(3);
                break;
            case -1:
                // !!! Write here again !!!
                flameLED.clear();
                return NAV_RIGHT;
            // It is alternative way to stop in front of the candle. Third way.
//            case 0:
//                stop();
//                return NAV_RIGHT;
        }

        // It is alternative way to stop in front of the candle.
//        if (flameSensor.flameVal > FLAME_NEAR_LIMIT) {
//            stop();
//            return PUT_OUT;
//        }

        lcd.print(0, "Value: " + flameSensor.flameVal);
        lcd.print(1, "Direction: " + flameSensor.flameDir);
        return CENTER;
    }

    private static int putOutState() {
        fan.setPower(16);
        wait(3000);
        fan.setPower(0);
        flameLED.clear();
        return WAIT;
    }

    //****** Motor Functions ******
    private static void move(int power, int delta) {
        lMotor.setPower(power + delta);
        rMotor.setPower(power - delta);
    }

    public static void rotateAngle(int angle) {
        if (angle < 0) {
            angle *= -1;
            rotate(TURN_VELO);
        }
        else rotate(TURN_VELO);

        wait((int)(angle * TURN_FACTOR));
    }

    private static void rotate(int velocity) {
        lMotor.setPower(-velocity);
        rMotor.setPower(velocity);
    }

    private static void bumperManuever(int bumper) {
        stop();
        if (bumper == LEFT_BUMPER) move(-GO_VELO, 4);
        else                       move(-GO_VELO, -4);
        wait(1000);
        stop();
    }

    private static void stop() {
        lMotor.stop();
        rMotor.stop();
        // We can also use break() function on behalf of stop() function.
    }

    //****** Sensor Functions ******
    private static float getDist(RangeFinder s) {
        s.ping();
        wait(25);
        return s.getDistanceCm();
    }

    // White-line sensor
    private static int getFloorTag() {
        if (lineSensor.sample() < LINE_LIMIT) {
            move(GO_VELO, 0);
            wait(500);
            stop();
            if (lineSensor.sample() < LINE_LIMIT)
                return LINE_TAG;
            else
                return CIRCLE_TAG;
        }
        else
            return NO_TAG;
    }


    //****** Auxiliary Functions ******
    private static void wait(int ms) {
        try {
            Thread.sleep(ms);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private static void displayState() {
        switch (state) {
            case WAIT:      lcd.print(0, "WAIT");      break;
            case NAV_RIGHT: lcd.print(0, "NAV_RIGHT"); break;
            case CENTER:    lcd.print(0, "CENTER");    break;
            case PUT_OUT:   lcd.print(0, "PUT_OUT");   break;
        }
    }
}
