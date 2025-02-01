#include <zumo_serial/zumo_serial_node.hpp>

///*********************************************************************************
/// Constructor
///*********************************************************************************
ZumoSerialNode::ZumoSerialNode()
               :rclcpp::Node("zumo_serial_node")                
{   
}

///*********************************************************************************
/// Deconstructor
///*********************************************************************************
ZumoSerialNode::~ZumoSerialNode() 
{
    try 
    {
        // Close the serial port
        m_ZumoStream.Close();
    } 
    catch(const std::exception& e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Exception occurred during cleanup: %s", e.what());
    }

    p_SerialThread->join();
    delete p_SerialThread;
}

///*********************************************************************************
/// Initialization
///*********************************************************************************
bool ZumoSerialNode::Init()
{
    // Initialize the parameters
    this->declare_parameter<std::string>(SERIAL_PORT_PARAMETER, SERIAL_PORT_DEFAULT);

    // Get the parameter value
    z_SerialPort = this->get_parameter(SERIAL_PORT_PARAMETER).as_string();
    RCLCPP_INFO(this->get_logger(), "serial port: %s\n", z_SerialPort.c_str());

    try 
    {
        // Serial port settings
        // Open the serial port
        m_ZumoStream.Open(z_SerialPort);

        // Set the baud rate of the serial port.
        m_ZumoStream.SetBaudRate(BaudRate::BAUD_115200);

        // Set the number of data bits.
        m_ZumoStream.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

        // Turn off hardware flow control.
        m_ZumoStream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

        // Disable parity.
        m_ZumoStream.SetParity(Parity::PARITY_NONE);

        // Set the number of stop bits.
        m_ZumoStream.SetStopBits(StopBits::STOP_BITS_1);  
    } 
    catch(const std::exception& e) 
    {
        RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
        return false;
    }   

    // ROS publisher setup
    m_ZumoSensorsPub = this->create_publisher<zumo_msgs::msg::ZumoSensors>("zumo_sensors", 10); 

    // serial read thread
    p_SerialThread = new boost::thread(boost::bind(&ZumoSerialNode::SerialDataThread, this));

    return true;
}

///*********************************************************************************
/// Read serial data
///*********************************************************************************
void ZumoSerialNode::ReadSerialData()
{
    try
    {
        std::string zReceivedData;
        std::getline(m_ZumoStream, zReceivedData);

        if(!zReceivedData.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "Received: %s", zReceivedData.c_str());

            istringstream iss(zReceivedData);
            string token;
            auto msg = zumo_msgs::msg::ZumoSensors();
            msg.header.stamp = this->now();
            int iSensorValues[SENSOR_COUNT];
            int i = 0;

            while(getline(iss, token, ',')) 
            {
                istringstream(token) >> iSensorValues[i];          
                i++;
            }

            // accelerometer
            msg.ax = iSensorValues[0];
            msg.ay = iSensorValues[1];
            msg.az = iSensorValues[2];
            
            // magnetometer
            msg.mx = iSensorValues[3];
            msg.my = iSensorValues[4];
            msg.mz = iSensorValues[5];

            // gyroscope
            msg.gx = iSensorValues[6];
            msg.gy = iSensorValues[7];
            msg.gz = iSensorValues[8];
            
            // encoder
            msg.enc_left = iSensorValues[9];
            msg.enc_right = iSensorValues[10];

            m_ZumoSensorsPub->publish(msg);
        }     
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception occurred during serial port reading: %s", e.what());
    }
}

///*********************************************************************************
/// Thread for reading serial data
///*********************************************************************************
void ZumoSerialNode::SerialDataThread()
{
    while(rclcpp::ok()) 
    {
        // Start reading serial port
        ReadSerialData();
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
}

    
