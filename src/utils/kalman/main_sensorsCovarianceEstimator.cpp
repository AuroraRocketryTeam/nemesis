/***
 * Temporary code for testing EKF with MPRLS and BNO055 sensors (the real main is main_orig.cpp)
 */

 static const uint8_t LM35_PIN = 0;

 #include <vector>
 #include <tuple>
 #include "./sensors/MPRLS/MPRLSSensor.hpp"
 #include "./sensors/BNO055/BNO055Sensor.hpp"
 #include "./utils/kalman/KalmanFilter.hpp"
 
 // Elements needed to update the kalman filter
 unsigned long lastTime = 0;
 
 KalmanFilter ekf;
 
 static MPRLSSensor mprls;
 static BNO055Sensor bno;
 
 // Vector to keep track of the position
 std::vector<std::tuple<float, float, float>> positionLog;
 
 // Data for covariance evaluation
 const int IT_SIZE = 200;
 int elements_counter = 0;
 std::vector<float> orientation_x(IT_SIZE);
 std::vector<float> orientation_y(IT_SIZE);
 std::vector<float> orientation_z(IT_SIZE);
 std::vector<float> angVelocity_x(IT_SIZE);
 std::vector<float> angVelocity_y(IT_SIZE);
 std::vector<float> angVelocity_z(IT_SIZE);
 std::vector<float> linearAccel_x(IT_SIZE);
 std::vector<float> linearAccel_y(IT_SIZE); 
 std::vector<float> linearAccel_z(IT_SIZE);
 std::vector<float> magnetometer_x(IT_SIZE);
 std::vector<float> magnetometer_y(IT_SIZE);
 std::vector<float> magnetometer_z(IT_SIZE);
 std::vector<float> accelerometer_x(IT_SIZE);
 std::vector<float> accelerometer_y(IT_SIZE);
 std::vector<float> accelerometer_z(IT_SIZE);
 std::vector<float> gravity_x(IT_SIZE);
 std::vector<float> gravity_y(IT_SIZE);
 std::vector<float> gravity_z(IT_SIZE);
 std::vector<float> quaternion_w(IT_SIZE);
 std::vector<float> quaternion_x(IT_SIZE);
 std::vector<float> quaternion_y(IT_SIZE);
 std::vector<float> quaternion_z(IT_SIZE);
 std::vector<float> temperature(IT_SIZE);
 std::vector<float> pressure(IT_SIZE);
 
 void setup()
 {
     mprls.init();
     bno.init();
     elements_counter = 0;
 
     Serial.begin(115200);
 }
 
 void loop()
 {
     // Retrieve data from MPRLS sensor
     auto mprlsDataOpt = mprls.getData();
     if (!mprlsDataOpt.has_value()) {
        Serial.println("MPRLS data not available");
        return;
     }
     auto mprlsData = mprlsDataOpt.value();
     float mprls_pressure = std::get<float>(mprlsData.getData("pressure").value());
 
     // Retrieve data from BNO055 sensor
     auto bnoDataOpt = bno.getData();
     if (!bnoDataOpt.has_value()) {
        Serial.println("BNO055 data not available");
        return;
     }
     auto bnoData = bnoDataOpt.value();
     
     float bno_orientation_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["x"]);
     float bno_orientation_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["y"]);
     float bno_orientation_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["z"]);
 
     float bno_angVelocity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["x"]);
     float bno_angVelocity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["y"]);
     float bno_angVelocity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["z"]);
 
     float bno_linearAccel_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["x"]);
     float bno_linearAccel_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["y"]);
     float bno_linearAccel_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["z"]);
     
     float bno_magnetometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["x"]);
     float bno_magnetometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["y"]);
     float bno_magnetometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["z"]);
     
     float bno_accelerometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["x"]);
     float bno_accelerometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["y"]);
     float bno_accelerometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["z"]);
     
     float bno_gravity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["x"]);
     float bno_gravity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["y"]);
     float bno_gravity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["z"]);
     
     float bno_temperature = static_cast<float>(std::get<int8_t>(bnoData.getData("board_temperature").value()));
     
     float bno_quaternion_w = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["w"]);
     float bno_quaternion_x = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["x"]);
     float bno_quaternion_y = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["y"]);
     float bno_quaternion_z = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["z"]);
     
     // Calculate the standard deviation
     if (elements_counter < IT_SIZE) {
         orientation_x[elements_counter] = bno_orientation_x;
         orientation_y[elements_counter] = bno_orientation_y;
         orientation_z[elements_counter] = bno_orientation_z;
         angVelocity_x[elements_counter] = bno_angVelocity_x;
         angVelocity_y[elements_counter] = bno_angVelocity_y;
         angVelocity_z[elements_counter] = bno_angVelocity_z;
         linearAccel_x[elements_counter] = bno_linearAccel_x;
         linearAccel_y[elements_counter] = bno_linearAccel_y;
         linearAccel_z[elements_counter] = bno_linearAccel_z;
         magnetometer_x[elements_counter] = bno_magnetometer_x;
         magnetometer_y[elements_counter] = bno_magnetometer_y;
         magnetometer_z[elements_counter] = bno_magnetometer_z;
         accelerometer_x[elements_counter] = bno_accelerometer_x;
         accelerometer_y[elements_counter] = bno_accelerometer_y;
         accelerometer_z[elements_counter] = bno_accelerometer_z;
         gravity_x[elements_counter] = bno_gravity_x;
         gravity_y[elements_counter] = bno_gravity_y;
         gravity_z[elements_counter] = bno_gravity_z;
         quaternion_w[elements_counter] = bno_quaternion_w;
         quaternion_x[elements_counter] = bno_quaternion_x;
         quaternion_y[elements_counter] = bno_quaternion_y;
         quaternion_z[elements_counter] = bno_quaternion_z;
         temperature[elements_counter] = static_cast<float>(bno_temperature);
         pressure[elements_counter] = mprls_pressure;
         
         elements_counter++;
         Serial.print("Elements counter: ");
         Serial.println(elements_counter);
     } else {
         float orientation_y_mean = 0;
         float orientation_z_mean = 0;
         float orientation_x_mean = 0;
         float angVelocity_x_mean = 0;
         float angVelocity_y_mean = 0;
         float angVelocity_z_mean = 0;
         float linearAccel_x_mean = 0;
         float linearAccel_y_mean = 0;
         float linearAccel_z_mean = 0;
         float magnetometer_x_mean = 0;
         float magnetometer_y_mean = 0;
         float magnetometer_z_mean = 0;
         float accelerometer_x_mean = 0;
         float accelerometer_y_mean = 0;
         float accelerometer_z_mean = 0;
         float gravity_x_mean = 0;
         float gravity_y_mean = 0;
         float gravity_z_mean = 0;
         float quaternion_w_mean = 0;
         float quaternion_x_mean = 0;
         float quaternion_y_mean = 0;
         float quaternion_z_mean = 0;
         float temperature_mean = 0;
         float pressure_mean = 0;
 
         for (int i=0; i<IT_SIZE; i++) {
             orientation_x_mean += orientation_x[i];
             orientation_y_mean += orientation_y[i];
             orientation_z_mean += orientation_z[i];
             angVelocity_x_mean += angVelocity_x[i];
             angVelocity_y_mean += angVelocity_y[i];
             angVelocity_z_mean += angVelocity_z[i];
             linearAccel_x_mean += linearAccel_x[i];
             linearAccel_y_mean += linearAccel_y[i];
             linearAccel_z_mean += linearAccel_z[i];
             magnetometer_x_mean += magnetometer_x[i];
             magnetometer_y_mean += magnetometer_y[i];
             magnetometer_z_mean += magnetometer_z[i];
             accelerometer_x_mean += accelerometer_x[i];
             accelerometer_y_mean += accelerometer_y[i];
             accelerometer_z_mean += accelerometer_z[i];
             gravity_x_mean += gravity_x[i];
             gravity_y_mean += gravity_y[i];
             gravity_z_mean += gravity_z[i];
             quaternion_w_mean += quaternion_w[i];
             quaternion_x_mean += quaternion_x[i];
             quaternion_y_mean += quaternion_y[i];
             quaternion_z_mean += quaternion_z[i];
             temperature_mean += temperature[i];
             pressure_mean += pressure[i];
         }
 
         orientation_x_mean /= IT_SIZE;
         orientation_y_mean /= IT_SIZE;
         orientation_z_mean /= IT_SIZE;
         angVelocity_x_mean /= IT_SIZE;
         angVelocity_y_mean /= IT_SIZE;
         angVelocity_z_mean /= IT_SIZE;
         linearAccel_x_mean /= IT_SIZE;
         linearAccel_y_mean /= IT_SIZE;
         linearAccel_z_mean /= IT_SIZE;
         magnetometer_x_mean /= IT_SIZE;
         magnetometer_y_mean /= IT_SIZE;
         magnetometer_z_mean /= IT_SIZE;
         accelerometer_x_mean /= IT_SIZE;
         accelerometer_y_mean /= IT_SIZE;
         accelerometer_z_mean /= IT_SIZE;
         gravity_x_mean /= IT_SIZE;
         gravity_y_mean /= IT_SIZE;
         gravity_z_mean /= IT_SIZE;
         quaternion_w_mean /= IT_SIZE;
         quaternion_x_mean /= IT_SIZE;
         quaternion_y_mean /= IT_SIZE;
         quaternion_z_mean /= IT_SIZE;
         temperature_mean /= IT_SIZE;
         pressure_mean /= IT_SIZE;
 
         float orientation_x_std = 0;
         float orientation_y_std = 0;
         float orientation_z_std = 0;
         float angVelocity_x_std = 0;
         float angVelocity_y_std = 0;
         float angVelocity_z_std = 0;
         float linearAccel_x_std = 0;
         float linearAccel_y_std = 0;
         float linearAccel_z_std = 0;
         float magnetometer_x_std = 0;
         float magnetometer_y_std = 0;
         float magnetometer_z_std = 0;
         float accelerometer_x_std = 0;
         float accelerometer_y_std = 0;
         float accelerometer_z_std = 0;
         float gravity_x_std = 0;
         float gravity_y_std = 0;
         float gravity_z_std = 0;
         float quaternion_w_std = 0;
         float quaternion_x_std = 0;
         float quaternion_y_std = 0;
         float quaternion_z_std = 0;
         float temperature_std = 0;
         float pressure_std = 0;
 
         for (int i=0; i<IT_SIZE; i++) {
             orientation_x_std += powf(orientation_x[i] - orientation_x_mean, 2);
             orientation_y_std += powf(orientation_y[i] - orientation_y_mean, 2);
             orientation_z_std += powf(orientation_z[i] - orientation_z_mean, 2);
             angVelocity_x_std += powf(angVelocity_x[i] - angVelocity_x_mean, 2);
             angVelocity_y_std += powf(angVelocity_y[i] - angVelocity_y_mean, 2);
             angVelocity_z_std += powf(angVelocity_z[i] - angVelocity_z_mean, 2);
             linearAccel_x_std += powf(linearAccel_x[i] - linearAccel_x_mean, 2);
             linearAccel_y_std += powf(linearAccel_y[i] - linearAccel_y_mean, 2);
             linearAccel_z_std += powf(linearAccel_z[i] - linearAccel_z_mean, 2);
             magnetometer_x_std += powf(magnetometer_x[i] - magnetometer_x_mean, 2);
             magnetometer_y_std += powf(magnetometer_y[i] - magnetometer_y_mean, 2);
             magnetometer_z_std += powf(magnetometer_z[i] - magnetometer_z_mean, 2);
             accelerometer_x_std += powf(accelerometer_x[i] - accelerometer_x_mean, 2);
             accelerometer_y_std += powf(accelerometer_y[i] - accelerometer_y_mean, 2);
             accelerometer_z_std += powf(accelerometer_z[i] - accelerometer_z_mean, 2);
             gravity_x_std += powf(gravity_x[i] - gravity_x_mean, 2);
             gravity_y_std += powf(gravity_y[i] - gravity_y_mean, 2);
             gravity_z_std += powf(gravity_z[i] - gravity_z_mean, 2);
             quaternion_w_std += powf(quaternion_w[i] - quaternion_w_mean, 2);
             quaternion_x_std += powf(quaternion_x[i] - quaternion_x_mean, 2);
             quaternion_y_std += powf(quaternion_y[i] - quaternion_y_mean, 2);
             quaternion_z_std += powf(quaternion_z[i] - quaternion_z_mean, 2);
             temperature_std += powf(temperature[i] - temperature_mean, 2);
             pressure_std += powf(pressure[i] - pressure_mean, 2);
         }
 
         orientation_x_std = orientation_x_std / (IT_SIZE-1);
         orientation_y_std = orientation_y_std / (IT_SIZE-1);
         orientation_z_std = orientation_z_std / (IT_SIZE-1);
         angVelocity_x_std = angVelocity_x_std / (IT_SIZE-1);
         angVelocity_y_std = angVelocity_y_std / (IT_SIZE-1);
         angVelocity_z_std = angVelocity_z_std / (IT_SIZE-1);
         linearAccel_x_std = linearAccel_x_std / (IT_SIZE-1);
         linearAccel_y_std = linearAccel_y_std / (IT_SIZE-1);
         linearAccel_z_std = linearAccel_z_std / (IT_SIZE-1);
         magnetometer_x_std = magnetometer_x_std / (IT_SIZE-1);
         magnetometer_y_std = magnetometer_y_std / (IT_SIZE-1);
         magnetometer_z_std = magnetometer_z_std / (IT_SIZE-1);
         accelerometer_x_std = accelerometer_x_std / (IT_SIZE-1);
         accelerometer_y_std = accelerometer_y_std / (IT_SIZE-1);
         accelerometer_z_std = accelerometer_z_std / (IT_SIZE-1);
         gravity_x_std = gravity_x_std / (IT_SIZE-1);
         gravity_y_std = gravity_y_std / (IT_SIZE-1);
         gravity_z_std = gravity_z_std / (IT_SIZE-1);
         quaternion_w_std = quaternion_w_std / (IT_SIZE-1);
         quaternion_x_std = quaternion_x_std / (IT_SIZE-1);
         quaternion_y_std = quaternion_y_std / (IT_SIZE-1);
         quaternion_z_std = quaternion_z_std / (IT_SIZE-1);
         temperature_std = temperature_std / (IT_SIZE-1);
         pressure_std = pressure_std / (IT_SIZE-1);
         
         Serial.println("Covariance Estimation:");
         Serial.println("Orientation X: " + String(orientation_x_std, 10));
         Serial.println("Orientation Y: " + String(orientation_y_std, 10));
         Serial.println("Orientation Z: " + String(orientation_z_std, 10));
         Serial.println("Angular Velocity X: " + String(angVelocity_x_std, 10));
         Serial.println("Angular Velocity Y: " + String(angVelocity_y_std, 10));
         Serial.println("Angular Velocity Z: " + String(angVelocity_z_std, 10));
         Serial.println("Linear Acceleration X: " + String(linearAccel_x_std, 10));
         Serial.println("Linear Acceleration Y: " + String(linearAccel_y_std, 10));
         Serial.println("Linear Acceleration Z: " + String(linearAccel_z_std, 10));
         Serial.println("Magnetometer X: " + String(magnetometer_x_std, 10));
         Serial.println("Magnetometer Y: " + String(magnetometer_y_std, 10));
         Serial.println("Magnetometer Z: " + String(magnetometer_z_std, 10));
         Serial.println("Accelerometer X: " + String(accelerometer_x_std, 10));
         Serial.println("Accelerometer Y: " + String(accelerometer_y_std, 10));
         Serial.println("Accelerometer Z: " + String(accelerometer_z_std, 10));
         Serial.println("Gravity X: " + String(gravity_x_std, 10));
         Serial.println("Gravity Y: " + String(gravity_y_std, 10));
         Serial.println("Gravity Z: " + String(gravity_z_std, 10));
         Serial.println("Quaternion W: " + String(quaternion_w_std, 10));
         Serial.println("Quaternion X: " + String(quaternion_x_std, 10));
         Serial.println("Quaternion Y: " + String(quaternion_y_std, 10));
         Serial.println("Quaternion Z: " + String(quaternion_z_std, 10));
         Serial.println("Temperature: " + String(temperature_std, 10));
         Serial.println("Pressure: " + String(pressure_std, 10));
         
         delay(10000);
    }
     delay(100);
 }