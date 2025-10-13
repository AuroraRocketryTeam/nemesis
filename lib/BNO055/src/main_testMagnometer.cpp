// #include <vector>
// #include <tuple>
// #include "../../sensors/MPRLS/MPRLSSensor.hpp"
// #include "../../sensors/BNO055/BNO055Sensor.hpp"
 
// static MPRLSSensor mprls;
// static BNO055Sensor bno;

// void setup()
// {
//     mprls.init();
//     bno.init();

//     Serial.begin(115200);
// }
 
// void loop()
// {
//     // Retrieve data from BNO055 sensor
//     auto bnoDataOpt = bno.getData();
//     if (!bnoDataOpt.has_value()) {
//         Serial.println("BNO055 data not available");
//         return;
//     }
//     auto bnoData = bnoDataOpt.value();

//     float bno_magnetometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["x"]);
//     float bno_magnetometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["y"]);
//     float bno_magnetometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["z"]);

//     Serial.println("Magnometer x: " + String(bno_magnetometer_x) + 
//                    ", y: " + String(bno_magnetometer_y) + 
//                    ", z: " + String(bno_magnetometer_z));
    
//     delay(10);
// }