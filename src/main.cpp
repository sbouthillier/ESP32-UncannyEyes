#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "config.h"


/**************************************************************
*   Definitions
***************************************************************/
#define USE_DMA

// A pixel buffer is used during eye rendering
#define BUFFER_SIZE 1024 // 128 to 1024 seems optimum

#ifdef USE_DMA
  #define BUFFERS 2      // 2 toggle buffers with DMA
#else
  #define BUFFERS 1      // 1 buffer for no DMA
#endif

/**************************************************************
*   Type definitions
***************************************************************/
typedef struct              
{
  int8_t  select;       // pin numbers for each eye's screen select line
  int8_t  wink;         // and wink button (or -1 if none) specified there,
  uint8_t rotation;     // also display rotation and the x offset
  int16_t xposition;    // position of eye on the screen
} eyeInfo_t;

// A simple state machine is used to control eye blinks/winks:
#define NOBLINK 0       // Not currently engaged in a blink
#define ENBLINK 1       // Eyelid is currently closing
#define DEBLINK 2       // Eyelid is currently opening
typedef struct 
{
  uint8_t  state;       // NOBLINK/ENBLINK/DEBLINK
  uint32_t duration;    // Duration of blink state (micros)
  uint32_t startTime;   // Time (micros) of last state change
} eyeBlink;

// One-per-eye structure
typedef struct 
{
  int16_t   tft_cs;     // Chip select pin for each display
  eyeBlink  blink;      // Current blink/wink state
  int16_t   xposition;  // x position of eye image
} eye_t;

/**************************************************************
*   Local variables
***************************************************************/
static TFT_eSPI tft;                            // A single instance is used for 1 or 2 displays

static uint16_t pbuffer[BUFFERS][BUFFER_SIZE]; // Pixel rendering buffer
static bool     dmaBuf   = 0;                  // DMA buffer selection

static eye_t eye[NUM_EYES];

// This table contains ONE LINE PER EYE.  The table MUST be present with
// this name and contain ONE OR MORE lines.  Each line contains THREE items:
// a pin number for the corresponding TFT/OLED display's SELECT line, a pin
// pin number for that eye's "wink" button (or -1 if not used), a screen
// rotation value (0-3) and x position offset for that eye.
#if (NUM_EYES == 2)
  eyeInfo_t eyeInfo[] = {
    { TFT1_CS, LH_WINK_PIN, TFT_1_ROT, EYE_1_XPOSITION }, // LEFT EYE chip select and wink pins, rotation and offset
    { TFT2_CS, RH_WINK_PIN, TFT_2_ROT, EYE_2_XPOSITION }, // RIGHT EYE chip select and wink pins, rotation and offset
  };
#else
  eyeInfo_t eyeInfo[] = {
    { TFT1_CS, LH_WINK_PIN, TFT_1_ROT, EYE_1_XPOSITION }, // EYE chip select and wink pins, rotation and offset
  };
#endif

#if !defined(LIGHT_PIN) || (LIGHT_PIN < 0)
// Autonomous iris motion uses a fractal behavior to similate both the major
// reaction of the eye plus the continuous smaller adjustments that occur.
static uint16_t oldIris = (IRIS_MIN + IRIS_MAX) / 2, newIris;
#endif

static uint32_t startTime;

/**************************************************************
*   Local Functions
***************************************************************/
static void initEyes(void);
static void updateEye(void);
static void drawEye(uint8_t  e, uint32_t iScale, uint32_t scleraX, uint32_t scleraY, uint32_t uT, uint32_t lT);
static void frame(uint16_t iScale);
static void split(int16_t  startValue, int16_t  endValue, uint32_t startTime, int32_t  duration, int16_t  range);  


void setup(void)
{
    Serial.begin(115200);
    Serial.println("Starting");

#if defined(DISPLAY_BACKLIGHT) && (DISPLAY_BACKLIGHT >= 0)
    // Enable backlight pin, initially off
    Serial.println("Backlight turned off");
    pinMode(DISPLAY_BACKLIGHT, OUTPUT);
    digitalWrite(DISPLAY_BACKLIGHT, LOW);
#endif

    // Initialise the eye(s), this will set all chip selects low for the tft.init()
    initEyes();

    // Initialise TFT
    Serial.println("Initialising displays");
    tft.init();

#ifdef USE_DMA
    Serial.println("Initialising DMA");
    tft.initDMA();
#endif

    // Raise chip select(s) so that displays can be individually configured
    digitalWrite(eye[0].tft_cs, HIGH);

    if (NUM_EYES > 1)
    {
        digitalWrite(eye[1].tft_cs, HIGH);
    }

    for (uint8_t e = 0; e < NUM_EYES; e++)
    {
        digitalWrite(eye[e].tft_cs, LOW);
        tft.setRotation(eyeInfo[e].rotation);
        tft.fillScreen(TFT_BLACK);
        digitalWrite(eye[e].tft_cs, HIGH);
    }

#if defined(DISPLAY_BACKLIGHT) && (DISPLAY_BACKLIGHT >= 0)
    Serial.println("Backlight now on!");
    analogWrite(DISPLAY_BACKLIGHT, BACKLIGHT_MAX);
#endif

    startTime = millis(); // For frame-rate calculation
}

void loop() 
{
    updateEye();
}


/*
* initEye
*
* Initialise eyes
*
*/
static void initEyes(void)
{
    Serial.println("Initialise eye objects");

    // Initialise eye objects based on eyeInfo list in config.h:
    for (uint8_t e = 0; e < NUM_EYES; e++)
    {
        Serial.print("Create display #");
        Serial.println(e);

        eye[e].tft_cs = eyeInfo[e].select;
        eye[e].blink.state = NOBLINK;
        eye[e].xposition = eyeInfo[e].xposition;

        pinMode(eye[e].tft_cs, OUTPUT);
        digitalWrite(eye[e].tft_cs, LOW);

        // Also set up an individual eye-wink pin if defined:
        if (eyeInfo[e].wink >= 0)
            pinMode(eyeInfo[e].wink, INPUT_PULLUP);
    }

#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
    pinMode(BLINK_PIN, INPUT_PULLUP); // Ditto for all-eyes blink pin
#endif
}

/*
* updateEye
*
* Update eyes
*
*/
static void updateEye(void)
{
#if defined(LIGHT_PIN) && (LIGHT_PIN >= 0) // Interactive iris

    int16_t v = analogRead(LIGHT_PIN); // Raw dial/photocell reading
#ifdef LIGHT_PIN_FLIP
    v = 1023 - v; // Reverse reading from sensor
#endif
    if (v < LIGHT_MIN)
        v = LIGHT_MIN; // Clamp light sensor range
    else if (v > LIGHT_MAX)
        v = LIGHT_MAX;
    v -= LIGHT_MIN; // 0 to (LIGHT_MAX - LIGHT_MIN)
#ifdef LIGHT_CURVE  // Apply gamma curve to sensor input?
    v = (int16_t)(pow((double)v / (double)(LIGHT_MAX - LIGHT_MIN),
                      LIGHT_CURVE) *
                  (double)(LIGHT_MAX - LIGHT_MIN));
#endif
    // And scale to iris range (IRIS_MAX is size at LIGHT_MIN)
    v = map(v, 0, (LIGHT_MAX - LIGHT_MIN), IRIS_MAX, IRIS_MIN);
#ifdef IRIS_SMOOTH // Filter input (gradual motion)
    static int16_t irisValue = (IRIS_MIN + IRIS_MAX) / 2;
    irisValue = ((irisValue * 15) + v) / 16;
    frame(irisValue);
#else  // Unfiltered (immediate motion)
    frame(v);
#endif // IRIS_SMOOTH

#else // Autonomous iris scaling -- invoke recursive function

    newIris = random(IRIS_MIN, IRIS_MAX);
    split(oldIris, newIris, micros(), 10000000L, IRIS_MAX - IRIS_MIN);
    oldIris = newIris;

#endif // LIGHT_PIN
}

/*
* drawEye
* 
* Eye rendering function
*
* Inputs:
*  e        Eye array index; 0 or 1 for left/right
*  iScale   Scale factor for iris
*  scleraX  First pixel X offset into sclera image
*  scleraY  First pixel Y offset into sclera image
*  uT       Upper eyelid threshold value
*  lT       Lower eyelid threshold value
*
*/
static void drawEye(uint8_t e, uint32_t iScale, uint32_t scleraX, uint32_t scleraY, uint32_t uT, uint32_t  lT)
{
    uint32_t screenX, screenY, scleraXsave;
    int32_t irisX, irisY;
    uint32_t p, a;
    uint32_t d;

    uint32_t pixels = 0;

    // Set up raw pixel dump to entire screen.  Although such writes can wrap
    // around automatically from end of rect back to beginning, the region is
    // reset on each frame here in case of an SPI glitch.
    digitalWrite(eye[e].tft_cs, LOW);

    tft.startWrite();
    tft.setAddrWindow(eye[e].xposition, 0, SCREEN_HEIGHT, SCREEN_WIDTH);

    // Now just issue raw 16-bit values for every pixel...

    scleraXsave = scleraX; // Save initial X value to reset on each line
    irisY = scleraY - (SCLERA_HEIGHT - IRIS_HEIGHT) / 2;

    // Eyelid image is left<>right swapped for two displays
    uint16_t lidX = 0;
    uint16_t dlidX = -1;

    if (e)
        dlidX = 1;

    for (screenY = 0; screenY < SCREEN_HEIGHT; screenY++, scleraY++, irisY++)
    {
        scleraX = scleraXsave;
        irisX = scleraXsave - (SCLERA_WIDTH - IRIS_WIDTH) / 2;

        if (e)
            lidX = 0;
        else
            lidX = SCREEN_WIDTH - 1;

        for (screenX = 0; screenX < SCREEN_WIDTH; screenX++, scleraX++, irisX++, lidX += dlidX)
        {
            if ((pgm_read_byte(lower + screenY * SCREEN_WIDTH + lidX) <= lT) ||
                (pgm_read_byte(upper + screenY * SCREEN_WIDTH + lidX) <= uT))
            { // Covered by eyelid
                p = 0;
            }
            else if ((irisY < 0) || (irisY >= IRIS_HEIGHT) ||
                     (irisX < 0) || (irisX >= IRIS_WIDTH))
            { // In sclera
                p = pgm_read_word(sclera + scleraY * SCLERA_WIDTH + scleraX);
            }
            else
            {                                                          // Maybe iris...
                p = pgm_read_word(polar + irisY * IRIS_WIDTH + irisX); // Polar angle/dist
                d = (iScale * (p & 0x7F)) / 240;                       // Distance (Y)

                if (d < IRIS_MAP_HEIGHT)
                {                                                     // Within iris area
                    a = (IRIS_MAP_WIDTH * (p >> 7)) / 512;            // Angle (X)
                    p = pgm_read_word(iris + d * IRIS_MAP_WIDTH + a); // Pixel = iris
                }
                else
                {                                                                 // Not in iris
                    p = pgm_read_word(sclera + scleraY * SCLERA_WIDTH + scleraX); // Pixel = sclera
                }
            }
            *(&pbuffer[dmaBuf][0] + pixels++) = p >> 8 | p << 8;

            if (pixels >= BUFFER_SIZE)
            {
                yield();
#ifdef USE_DMA
                tft.pushPixelsDMA(&pbuffer[dmaBuf][0], pixels);
                dmaBuf = !dmaBuf;
#else
                tft.pushPixels(pbuffer, pixels);
#endif
                pixels = 0;
            }
        }
    }

    if (pixels)
    {
#ifdef USE_DMA
        tft.pushPixelsDMA(&pbuffer[dmaBuf][0], pixels);
#else
        tft.pushPixels(pbuffer, pixels);
#endif
    }
    tft.endWrite();
    digitalWrite(eye[e].tft_cs, HIGH);
}

// EYE ANIMATION -----------------------------------------------------------

// Ease in/out curve for eye movements 3*t^2-2*t^3
const uint8_t ease[] = { 
  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,                    // T
  3,  3,  4,  4,  4,  5,  5,  6,  6,  7,  7,  8,  9,  9, 10, 10,                    // h
  11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23,                   // x
  24, 25, 26, 27, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 39,                   // 2
  40, 41, 42, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 56, 57, 58,                   // A
  60, 61, 62, 63, 65, 66, 67, 69, 70, 72, 73, 74, 76, 77, 78, 80,                   // l
  81, 83, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 98, 100, 101, 103,                // e
  104, 106, 107, 109, 110, 112, 113, 115, 116, 118, 119, 121, 122, 124, 125, 127,   // c
  128, 130, 131, 133, 134, 136, 137, 139, 140, 142, 143, 145, 146, 148, 149, 151,   // J
  152, 154, 155, 157, 158, 159, 161, 162, 164, 165, 167, 168, 170, 171, 172, 174,   // a
  175, 177, 178, 179, 181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 195,   // c
  197, 198, 199, 201, 202, 203, 204, 205, 207, 208, 209, 210, 211, 213, 214, 215,   // o
  216, 217, 218, 219, 220, 221, 222, 224, 225, 226, 227, 228, 228, 229, 230, 231,   // b
  232, 233, 234, 235, 236, 237, 237, 238, 239, 240, 240, 241, 242, 243, 243, 244,   // s
  245, 245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 251, 251, 251, 252, 252,   // o
  252, 253, 253, 253, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255    // n
};

#ifdef AUTOBLINK
uint32_t timeOfLastBlink = 0L, timeToNextBlink = 0L;
#endif

/*
* frame
*
* Process motion for a single frame of left or right eye
*
* Inputs:
*  iScale   Iris scale (0-1023)
*
*/
static void frame(uint16_t iScale)
{
    static uint32_t frames = 0;  // Used in frame rate calculation
    static uint8_t eyeIndex = 0; // eye[] array counter
    int16_t eyeX, eyeY;
    uint32_t t = micros(); // Time at start of function

    if (!(++frames & 255))
    { // Every 256 frames...
        float elapsed = (millis() - startTime) / 1000.0;

        if (elapsed)
        {
            Serial.println((uint16_t)(frames / elapsed)); // Print FPS
        }
    }

    if (++eyeIndex >= NUM_EYES)
    {
        eyeIndex = 0; // Cycle through eyes, 1 per call
    }
        // X/Y movement

#if defined(JOYSTICK_X_PIN) && (JOYSTICK_X_PIN >= 0) && \
    defined(JOYSTICK_Y_PIN) && (JOYSTICK_Y_PIN >= 0)

    // Read X/Y from joystick, constrain to circle
    int16_t dx, dy;
    int32_t d;

    eyeX = analogRead(JOYSTICK_X_PIN); // Raw (unclipped) X/Y reading
    eyeY = analogRead(JOYSTICK_Y_PIN);
#ifdef JOYSTICK_X_FLIP
    eyeX = 1023 - eyeX;
#endif
#ifdef JOYSTICK_Y_FLIP
    eyeY = 1023 - eyeY;
#endif
    dx = (eyeX * 2) - 1023; // A/D exact center is at 511.5.  Scale coords
    dy = (eyeY * 2) - 1023; // X2 so range is -1023 to +1023 w/center at 0.

    if ((d = (dx * dx + dy * dy)) > (1023 * 1023))
    {                                        // Outside circle
        d = (int32_t)sqrt((float)d);         // Distance from center
        eyeX = ((dx * 1023 / d) + 1023) / 2; // Clip to circle edge,
        eyeY = ((dy * 1023 / d) + 1023) / 2; // scale back to 0-1023
    }

#else  // Autonomous X/Y eye motion
    // Periodically initiates motion to a new random point, random speed,
    // holds there for random period until next motion.

    static bool eyeInMotion = false;
    static int16_t eyeOldX = 512, eyeOldY = 512, eyeNewX = 512, eyeNewY = 512;
    static uint32_t eyeMoveStartTime = 0L;
    static int32_t eyeMoveDuration = 0L;

    int32_t dt = t - eyeMoveStartTime; // uS elapsed since last eye event

    if (eyeInMotion)
    { // Currently moving?
        if (dt >= eyeMoveDuration)
        {                                      // Time up?  Destination reached.
            eyeInMotion = false;               // Stop moving
            eyeMoveDuration = random(3000000); // 0-3 sec stop
            eyeMoveStartTime = t;              // Save initial time of stop
            eyeX = eyeOldX = eyeNewX;          // Save position
            eyeY = eyeOldY = eyeNewY;
        }
        else
        {                                                       // Move time's not yet fully elapsed -- interpolate position
            int16_t e = ease[255 * dt / eyeMoveDuration] + 1;   // Ease curve

            eyeX = eyeOldX + (((eyeNewX - eyeOldX) * e) / 256); // Interp X
            eyeY = eyeOldY + (((eyeNewY - eyeOldY) * e) / 256); // and Y
        }
    }
    else
    { // Eye stopped
        eyeX = eyeOldX;
        eyeY = eyeOldY;

        if (dt > eyeMoveDuration)
        { // Time up?  Begin new move.
            int16_t dx, dy;
            uint32_t d;

            do
            { // Pick new dest in circle
                eyeNewX = random(1024);
                eyeNewY = random(1024);
                dx = (eyeNewX * 2) - 1023;
                dy = (eyeNewY * 2) - 1023;
            } while ((d = (dx * dx + dy * dy)) > (1023 * 1023)); // Keep trying

            eyeMoveDuration = random(72000, 144000);             // ~1/14 - ~1/7 sec
            eyeMoveStartTime = t;                                // Save initial time of move
            eyeInMotion = true;                                  // Start move on next frame
        }
    }
#endif // JOYSTICK_X_PIN etc.

    // Blinking
#ifdef AUTOBLINK
    // Similar to the autonomous eye movement above -- blink start times
    // and durations are random (within ranges).
    if ((t - timeOfLastBlink) >= timeToNextBlink)
    { // Start new blink?
        timeOfLastBlink = t;
        uint32_t blinkDuration = random(36000, 72000); // ~1/28 - ~1/14 sec

        // Set up durations for both eyes (if not already winking)
        for (uint8_t e = 0; e < NUM_EYES; e++)
        {
            if (eye[e].blink.state == NOBLINK)
            {
                eye[e].blink.state = ENBLINK;
                eye[e].blink.startTime = t;
                eye[e].blink.duration = blinkDuration;
            }
        }
        timeToNextBlink = blinkDuration * 3 + random(4000000);
    }
#endif

    if (eye[eyeIndex].blink.state)
    { // Eye currently blinking?
        // Check if current blink state time has elapsed
        if ((t - eye[eyeIndex].blink.startTime) >= eye[eyeIndex].blink.duration)
        {
            // Yes -- increment blink state, unless...
            if ((eye[eyeIndex].blink.state == ENBLINK) && ( // Enblinking and...
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
                (digitalRead(BLINK_PIN) == LOW) || // blink or wink held...
#endif
                ((eyeInfo[eyeIndex].wink >= 0) && digitalRead(eyeInfo[eyeIndex].wink) == LOW)))
            {
                // Don't advance state yet -- eye is held closed instead
            }
            else
            { // No buttons, or other state...
                if (++eye[eyeIndex].blink.state > DEBLINK)
                {                                        // Deblinking finished?
                    eye[eyeIndex].blink.state = NOBLINK; // No longer blinking
                }
                else
                {                                      // Advancing from ENBLINK to DEBLINK mode
                    eye[eyeIndex].blink.duration *= 2; // DEBLINK is 1/2 ENBLINK speed
                    eye[eyeIndex].blink.startTime = t;
                }
            }
        }
    }
    else
    { // Not currently blinking...check buttons!
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
        if (digitalRead(BLINK_PIN) == LOW)
        {
            // Manually-initiated blinks have random durations like auto-blink
            uint32_t blinkDuration = random(36000, 72000);
            for (uint8_t e = 0; e < NUM_EYES; e++)
            {
                if (eye[e].blink.state == NOBLINK)
                {
                    eye[e].blink.state = ENBLINK;
                    eye[e].blink.startTime = t;
                    eye[e].blink.duration = blinkDuration;
                }
            }
        }
        else
#endif
        if ((eyeInfo[eyeIndex].wink >= 0) && (digitalRead(eyeInfo[eyeIndex].wink) == LOW))
        { // Wink!
            eye[eyeIndex].blink.state = ENBLINK;
            eye[eyeIndex].blink.startTime = t;
            eye[eyeIndex].blink.duration = random(45000, 90000);
        }
    }

    // Process motion, blinking and iris scale into renderable values

    // Scale eye X/Y positions (0-1023) to pixel units used by drawEye()
    eyeX = map(eyeX, 0, 1023, 0, SCLERA_WIDTH  - DISPLAY_SIZE);
    eyeY = map(eyeY, 0, 1023, 0, SCLERA_HEIGHT - DISPLAY_SIZE);

    // Horizontal position is offset so that eyes are very slightly crossed
    // to appear fixated (converged) at a conversational distance.  Number
    // here was extracted from my posterior and not mathematically based.
    // I suppose one could get all clever with a range sensor, but for now...
    if (NUM_EYES > 1)
    {
        if (eyeIndex == 1)
            eyeX += 4;
        else
            eyeX -= 4;
    }
    if (eyeX > (SCLERA_WIDTH - DISPLAY_SIZE))
        eyeX = (SCLERA_WIDTH - DISPLAY_SIZE);

    // Eyelids are rendered using a brightness threshold image.  This same
    // map can be used to simplify another problem: making the upper eyelid
    // track the pupil (eyes tend to open only as much as needed -- e.g. look
    // down and the upper eyelid drops).  Just sample a point in the upper
    // lid map slightly above the pupil to determine the rendering threshold.
    static uint8_t uThreshold = DISPLAY_SIZE;
    uint8_t lThreshold, n;

#ifdef TRACKING
    int16_t sampleX = SCLERA_WIDTH / 2 - (eyeX / 2), // Reduce X influence
        sampleY = SCLERA_HEIGHT / 2 - (eyeY + IRIS_HEIGHT / 4);
    // Eyelid is slightly asymmetrical, so two readings are taken, averaged
    if (sampleY < 0)
        n = 0;
    else
        n = (pgm_read_byte(upper + sampleY * SCREEN_WIDTH + sampleX) +
             pgm_read_byte(upper + sampleY * SCREEN_WIDTH + (SCREEN_WIDTH - 1 - sampleX))) /
            2;
    uThreshold = (uThreshold * 3 + n) / 4; // Filter/soften motion
    // Lower eyelid doesn't track the same way, but seems to be pulled upward
    // by tension from the upper lid.
    lThreshold = 254 - uThreshold;

#else // No tracking -- eyelids full open unless blink modifies them
    uThreshold = lThreshold = 0;
#endif

    // The upper/lower thresholds are then scaled relative to the current
    // blink position so that blinks work together with pupil tracking.
    if (eye[eyeIndex].blink.state)
    { // Eye currently blinking?
        uint32_t s = (t - eye[eyeIndex].blink.startTime);

        if (s >= eye[eyeIndex].blink.duration)
        {
            s = 255; // At or past blink end
        }
        else
        {
            s = 255 * s / eye[eyeIndex].blink.duration; // Mid-blink
        }

        s = (eye[eyeIndex].blink.state == DEBLINK) ? 1 + s : 256 - s;

        n = (uThreshold * s + 254 * (257 - s)) / 256;
        lThreshold = (lThreshold * s + 254 * (257 - s)) / 256;
    }
    else
    {
        n = uThreshold;
    }

    // Pass all the derived values to the eye-rendering function:
    drawEye(eyeIndex, iScale, eyeX, eyeY, n, lThreshold);
}

// AUTONOMOUS IRIS SCALING (if no photocell or dial) -----------------------

#if !defined(LIGHT_PIN) || (LIGHT_PIN < 0)

// Autonomous iris motion uses a fractal behavior to similate both the major
// reaction of the eye plus the continuous smaller adjustments that occur.

/*
* split
* 
* Subdivides motion path into two sub-paths with randomization
* startValue    Iris scale value (IRIS_MIN to IRIS_MAX) at start
* endValue      Iris scale value at end
* startTime     micros() at start
* duration      Start-to-end time, in microseconds
* range         Allowable scale value variance when subdividing
*
*/
static void split(int16_t startValue, int16_t endValue, uint32_t startTime, int32_t duration, int16_t range)
{ 
    // Allowable scale value variance when subdividing
    if (range >= 8)
    {                  // Limit subdvision count, because recursion
        range /= 2;    // Split range & time in half for subdivision,
        duration /= 2; // then pick random center point within range:

        int16_t midValue = (startValue + endValue - range) / 2 + random(range);
        uint32_t midTime = startTime + duration;

        split(startValue, midValue, startTime, duration, range); // First half
        split(midValue, endValue, midTime, duration, range);     // Second half
    }
    else
    {               // No more subdivisons, do iris motion...
        int32_t dt; // Time (micros) since start of motion
        int16_t v;  // Interim value

        while ((dt = (micros() - startTime)) < duration)
        {
            v = startValue + (((endValue - startValue) * dt) / duration);

            if (v < IRIS_MIN)
            {
                v = IRIS_MIN; // Clip just in case
            }
            else if (v > IRIS_MAX)
            {
                v = IRIS_MAX;
            }

            frame(v); // Draw frame w/interim iris scale value
        }
    }
}
#endif // !LIGHT_PIN
