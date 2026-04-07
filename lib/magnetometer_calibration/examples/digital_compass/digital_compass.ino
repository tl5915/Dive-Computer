/*
  This example demonstrates how to calibrate a magnetometer (e.g., QMC5883L)
  and apply calibration to calculate accurate compass headings.
  
  This library calibrates for hard iron and soft iron distortions using
  ellipsoid fitting, based on the work of sailboatinstruments1.
  https://sites.google.com/view/sailboatinstruments1/a-download-magneto-v1-2

  The model fallbacks to a simpler Min/Max diagonal method if ellipsoid fitting fails,
  usually due to insufficient or poor-quality calibration samples.

  Steps:
    1. Find your local magnetic field strength and convert nT to Gauss (1 Gauss = 100,000 nT):
    https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm

    2. Configure reference magnetic field and magnetometer LSB sensitivity

    3. Collect raw magnetometer samples over a period (rotate device in all orientations)

    3. Run calibration and store the resulting matrices

    4. Apply calibration matrices to raw readings to get corrected compass heading

    5. Apply tilt compensation using accelerometer data (Optional)
*/

#include <vector>
#include <magnetometer_calibration.h>

// ---- Configuration Constants ----
const float REFERENCE_FIELD_GAUSS = 0.49f;        // UK average magnetic field (49,000 nT)
const float LSB_PER_GAUSS = 3000.0f;              // QMC5883L at +/- 8 Gauss range: 3000 LSB/Gauss
const uint32_t CALIBRATION_DURATION_MS = 120000;  // 120 seconds for calibration sample collection
const uint32_t MAX_SAMPLES = 10000;               // Maximum number of samples to collect

// ---- Global State ----
std::vector<float> magnetometer_samples;  // Data input style: flattened vector [x, y, z, x, y, z, ...]
CompassCalibrationMatrices calibration_matrices = {};
CompassCalibrationFitMethod last_fit_method = CompassCalibrationFitMethod::None;


void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Configure the reference magnetic field and sensor sensitivity
    compassConfigureReferenceField(REFERENCE_FIELD_GAUSS, LSB_PER_GAUSS);  // Set reference field and sensitivity
    Serial.print("Reference field: ");
    Serial.print(compassReferenceFieldGauss());                            // Get reference field in Gauss
    Serial.println(" Gauss");
    Serial.print("Sensitivity: ");
    Serial.print(compassLsbPerGauss());                                    // Get magnetometer sensitivity in LSB/Gauss
    Serial.println(" LSB/Gauss");
    delay(1000);
}


void loop() {
    // Press C to start calibration
    if (Serial.available()) {
        char cmd = Serial.read();
        
        if (cmd == 'c' || cmd == 'C') {
            Serial.println("Starting calibration sample collection...");
            Serial.println("Rotate in all directions for 60 seconds");
            collectCalibrationSamples();  // Collect raw magnetometer readings for calibration
            
            Serial.println("Computing calibration...");
            bool cal_ok = runCalibration();  // Compute calibration matrices returning success/failure
            
            if (cal_ok) {
                Serial.println("Calibration done!");
                printCalibrationResults();  // Print the calibration results
                Serial.println("Reading calibrated compass heading:");
                startHeadingDemo();
            } else {
                Serial.println("Calibration failed!");
            }
        }
    }
}


/*
 * Collect raw magnetometer samples over CALIBRATION_DURATION_MS milliseconds.
 * In a real application, these would come from your magnetometer.
 */
void collectCalibrationSamples() {
    magnetometer_samples.reserve(MAX_SAMPLES * 3);  // Reserve memory for 3 floats per sample (x, y, z)
    
    uint32_t start_ms = millis();
    uint32_t count = 0;
    
    while ((millis() - start_ms) < CALIBRATION_DURATION_MS) {
        // ---- REPLACE THIS WITH YOUR MAGNETOMETER READ ----
        float raw_x = readMagX();  // Your sensor read function
        float raw_y = readMagY();
        float raw_z = readMagZ();
        
        magnetometer_samples.push_back(raw_x);
        magnetometer_samples.push_back(raw_y);
        magnetometer_samples.push_back(raw_z);
        count++;
        
        // Non-blocking polling delay
        delay(10);  // Appropriate time depends on sensor output rate
    }
    
    Serial.print("Sample count: ");
    Serial.println(count);
}


/*
 * Run calibration using the collected samples.
 * First attempts ellipsoid fitting; if that fails, falls back to Min/Max.
 */
bool runCalibration() {
    size_t sample_count = magnetometer_samples.size() / 3;
    
    if (sample_count < 50) {
        Serial.println("Too few samples");
        return false;
    }
    
    // Run calibration
    bool success = compassCalibrateFromSamplesWithFallback(
        magnetometer_samples.data(),
        sample_count,
        &calibration_matrices,
        &last_fit_method
    );
    
    if (!success) {
        Serial.println("Error: Calibration failed");
        return false;
    }
    
    // Apply the calibration matrices
    if (!compassSetCalibrationMatrices(&calibration_matrices)) {
        Serial.println("Error: Failed to set calibration matrices");
        return false;
    }
    
    return true;
}


/*
 * Print detailed calibration results.
 */
void printCalibrationResults() {
    Serial.println("Calibration Results");
    
    // Print calibration method used
    Serial.print("Method used: ");
    switch (last_fit_method) {
        case CompassCalibrationFitMethod::Ellipsoid:
            Serial.println("Ellipsoid");
            break;
        case CompassCalibrationFitMethod::MinMax:
            Serial.println("Min/Max");
            break;
        case CompassCalibrationFitMethod::Error:
            Serial.println("Error");
            break;
        case CompassCalibrationFitMethod::None:
        default:
            Serial.println("None");
            break;
    }
    
    // Print hard iron (bias) offsets
    Serial.println("Hard Iron (Bias) Offsets:");
    Serial.print("  X: ");
    Serial.println(calibration_matrices.hard_iron[0], 4);
    Serial.print("  Y: ");
    Serial.println(calibration_matrices.hard_iron[1], 4);
    Serial.print("  Z: ");
    Serial.println(calibration_matrices.hard_iron[2], 4);
    
    // Print soft iron (scale) matrix
    Serial.println("Soft Iron (Scale) Matrix:");
    Serial.print("  [");
    Serial.print(calibration_matrices.soft_iron[0], 3);
    Serial.print(" ");
    Serial.print(calibration_matrices.soft_iron[1], 3);
    Serial.print(" ");
    Serial.print(calibration_matrices.soft_iron[2], 3);
    Serial.println("]");
    Serial.print("  [");
    Serial.print(calibration_matrices.soft_iron[3], 3);
    Serial.print(" ");
    Serial.print(calibration_matrices.soft_iron[4], 3);
    Serial.print(" ");
    Serial.print(calibration_matrices.soft_iron[5], 3);
    Serial.println("]");
    Serial.print("  [");
    Serial.print(calibration_matrices.soft_iron[6], 3);
    Serial.print(" ");
    Serial.print(calibration_matrices.soft_iron[7], 3);
    Serial.print(" ");
    Serial.print(calibration_matrices.soft_iron[8], 3);
    Serial.println("]");
    
    // Print calibration quality
    Serial.print("Fitted Field Magnitude: ");
    Serial.print(calibration_matrices.fitted_field_lsb);
    Serial.println(" LSB");
}


/*
 * Show compass heading readout with the applied calibration.
 */
void startHeadingDemo() {
    while (true) {
        // Read raw magnetometer data
        float raw_mag[3] = {readMagX(), readMagY(), readMagZ()};

        // Apply calibration only (no tilt compensation)
        float cal_mag[3] = {0, 0, 0};
        compassApplyCalibration(raw_mag, cal_mag);
        float heading = compassHeadingFromMagneticVector(cal_mag);
        
        // Apply calibration and tilt compensation (with reading from accelerometer)
        float accel_x = readAccelX();  // Your sensor read function
        float accel_y = readAccelY();
        float accel_z = readAccelZ();
        float accel[3] = {accel_x, accel_y, accel_z};
        float tilt_comp_mag[3] = {0, 0, 0};
        compassApplyCalibrationAndTiltCompensation(raw_mag, accel, tilt_comp_mag);
        float heading_tilt = compassHeadingFromMagneticVector(tilt_comp_mag);
        
        // Print heading
        static uint32_t last_print = 0;
        if ((millis() - last_print) > 100) {
            Serial.print("Calibrated heading: ");
            Serial.println(heading, 1);
            Serial.print("Calibrated + tilt compensated heading: ");
            Serial.println(heading_tilt, 1);
            last_print = millis();
        }
        delay(100);
    }
}


// ---- Stub Sensor Read Functions ----
// Replace these with your actual magnetometer calls
float readMagX() {
    return 100.0f + random(-50, 50);  // Dummy data
}
float readMagY() {
    return 80.0f + random(-50, 50);
}
float readMagZ() {
    return 120.0f + random(-50, 50);
}
// Optional: IMU data for tilt compensation
float readAccelX() {
    return 100.0f + random(-50, 50);
}
float readAccelY() {
    return 100.0f + random(-50, 50);
}
float readAccelZ() {
    return 100.0f + random(-50, 50);
}