#include "E220LoRaTransmitter.hpp"

ResponseStatusContainer E220LoRaTransmitter::init()
{
    transmitter.begin();
    auto configurationStatus = this->getConfiguration();
    auto configuration = *(Configuration *)configurationStatus.data;

    configuration.ADDL = 0x03;
    configuration.ADDH = 0x00;

    configuration.CHAN = 23;

    configuration.SPED.uartBaudRate = UART_BPS_9600;
    configuration.SPED.airDataRate = AIR_DATA_RATE_111_625;
    configuration.SPED.uartParity = MODE_00_8N1;

    configuration.OPTION.subPacketSetting = SPS_200_00;
    configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
    configuration.OPTION.transmissionPower = POWER_22;

    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
    configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

    configurationStatus.close();
    return this->configure(configuration);
}

ResponseStatusContainer E220LoRaTransmitter::init(Configuration config)
{
    transmitter.begin();
    return this->configure(config);
}

ResponseStatusContainer E220LoRaTransmitter::transmit(std::variant<char *, String, std::string, nlohmann::json> data)
{
    // Extract the data from the variant
    String dataString;
    if (std::holds_alternative<char *>(data))
        dataString = String(std::get<char *>(data));
    else if (std::holds_alternative<String>(data))
        dataString = std::get<String>(data);
    else if (std::holds_alternative<std::string>(data))
        dataString = String(std::get<std::string>(data).c_str());
    else if (std::holds_alternative<nlohmann::json>(data))
        dataString = String(std::get<nlohmann::json>(data).dump().c_str());

    // Split and send data in chunks of up to 200 bytes
    const size_t MAX_CHUNK_SIZE = 199;
    for (size_t start = 0; start < dataString.length(); start += MAX_CHUNK_SIZE)
    {
        String chunk = dataString.substring(start, start + MAX_CHUNK_SIZE);
        auto sendResponse = transmitter.sendFixedMessage(LORA_DESTINATION_ADDH, LORA_DESTINATION_ADDL, LORA_CHANNEL, chunk);

        // If any chunk fails to send, store the error and exit the loop
        if (sendResponse.code != E220_SUCCESS)
        {
            return ResponseStatusContainer(sendResponse.code, sendResponse.getResponseDescription());
        }
    }
    return ResponseStatusContainer(E220_SUCCESS, "Data sent successfully.");
}

ResponseStatusContainer E220LoRaTransmitter::configure(Configuration configuration)
{
    auto response = transmitter.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    return ResponseStatusContainer(response.code, response.getResponseDescription());
}

ResponseStructContainer E220LoRaTransmitter::getConfiguration()
{
    return transmitter.getConfiguration();
}

String E220LoRaTransmitter::getConfigurationString(Configuration configuration) const
{
    String result;
    result += "----------------------------------------\n";

    result += "HEAD : " + String(configuration.COMMAND, HEX) + " " + String(configuration.STARTING_ADDRESS, HEX) + " " + String(configuration.LENGHT, HEX) + "\n\n";
    result += "AddH : " + String(configuration.ADDH, HEX) + "\n";
    result += "AddL : " + String(configuration.ADDL, HEX) + "\n\n";
    result += "Chan : " + String(configuration.CHAN, DEC) + " -> " + configuration.getChannelDescription() + "\n\n";
    result += "SpeedParityBit     : " + String(configuration.SPED.uartParity, BIN) + " -> " + configuration.SPED.getUARTParityDescription() + "\n";
    result += "SpeedUARTDatte     : " + String(configuration.SPED.uartBaudRate, BIN) + " -> " + configuration.SPED.getUARTBaudRateDescription() + "\n";
    result += "SpeedAirDataRate   : " + String(configuration.SPED.airDataRate, BIN) + " -> " + configuration.SPED.getAirDataRateDescription() + "\n\n";
    result += "OptionSubPacketSett: " + String(configuration.OPTION.subPacketSetting, BIN) + " -> " + configuration.OPTION.getSubPacketSetting() + "\n";
    result += "OptionTranPower    : " + String(configuration.OPTION.transmissionPower, BIN) + " -> " + configuration.OPTION.getTransmissionPowerDescription() + "\n";
    result += "OptionRSSIAmbientNo: " + String(configuration.OPTION.RSSIAmbientNoise, BIN) + " -> " + configuration.OPTION.getRSSIAmbientNoiseEnable() + "\n\n";
    result += "TransModeWORPeriod : " + String(configuration.TRANSMISSION_MODE.WORPeriod, BIN) + " -> " + configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription() + "\n";
    result += "TransModeEnableLBT : " + String(configuration.TRANSMISSION_MODE.enableLBT, BIN) + " -> " + configuration.TRANSMISSION_MODE.getLBTEnableByteDescription() + "\n";
    result += "TransModeEnableRSSI: " + String(configuration.TRANSMISSION_MODE.enableRSSI, BIN) + " -> " + configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription() + "\n";
    result += "TransModeFixedTrans: " + String(configuration.TRANSMISSION_MODE.fixedTransmission, BIN) + " -> " + configuration.TRANSMISSION_MODE.getFixedTransmissionDescription() + "\n";

    result += "----------------------------------------\n";

    return result;
}

template <typename T>
T E220LoRaTransmitter::getBpsValue(const std::map<int, T>& baudRateMap, T defaultValue) const
{
    auto it = baudRateMap.find(SERIAL_BAUD_RATE);
    if (it != baudRateMap.end())
    {
        return it->second;
    }
    else
    {
        return defaultValue;
    }
}

UART_BPS_RATE E220LoRaTransmitter::getBpsRate() const
{
    static const std::map<int, UART_BPS_RATE> baudRateMap = {
        {1200, UART_BPS_RATE_1200},
        {2400, UART_BPS_RATE_2400},
        {4800, UART_BPS_RATE_4800},
        {9600, UART_BPS_RATE_9600},
        {19200, UART_BPS_RATE_19200},
        {38400, UART_BPS_RATE_38400},
        {57600, UART_BPS_RATE_57600},
        {115200, UART_BPS_RATE_115200}};

    return getBpsValue(baudRateMap, UART_BPS_RATE_9600);
}

UART_BPS_TYPE E220LoRaTransmitter::getBpsType() const
{
    static const std::map<int, UART_BPS_TYPE> baudRateMap = {
        {1200, UART_BPS_1200},
        {2400, UART_BPS_2400},
        {4800, UART_BPS_4800},
        {9600, UART_BPS_9600},
        {19200, UART_BPS_19200},
        {38400, UART_BPS_38400},
        {57600, UART_BPS_57600},
        {115200, UART_BPS_115200}};

    return getBpsValue(baudRateMap, UART_BPS_9600);
}