#include "its_application.hpp"
#include <vanetza/btp/ports.hpp>
#include "BIT_STRING.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/denm.hpp>
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/asn1/spatem.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <iostream>
#include <boost/asio/io_service.hpp>
#include <boost/asio.hpp>
#include <boost/units/io.hpp>

#include "nlohmann/json.hpp"

// This is a very simple CA application sending CAMs at a fixed rate.

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;
namespace asio = boost::asio;


ITSApplication::ITSApplication(PositionProvider& positioning, Runtime& rt, asio::io_service& io_service, unsigned short denm_port) :
    positioning_(positioning), runtime_(rt), cam_interval_(seconds(1)),
    denm_socket(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), int(9001))),
    cam_socket(io_service)
{
    schedule_timer();    
    this->station_id = 1;
    this->server_port = 9000;
    this->serverIP = strdup("192.168.1.100");
    this->start_receive();
	
}

void print_indentedSpatem(std::ostream& os, const vanetza::asn1::Spatem& message, const std::string& indent, unsigned level) {
    auto prefix = [&](const char* field) -> std::ostream& {
        for (unsigned i = 0; i < level; ++i) {
            os << indent;
        }
        os << field << ": ";
        return os;
    };

    // Print ITS PDU Header
    const ItsPduHeader_t& header = message->header;
    prefix("ITS PDU Header") << "\n";
    ++level;
    prefix("Protocol Version") << static_cast<int>(header.protocolVersion) << "\n";
    prefix("Message ID") << static_cast<int>(header.messageID) << "\n";
    prefix("Station ID") << header.stationID << "\n";
    --level;

    // SPATEM content
    const SPAT_t& spatem = message->spat;

    prefix("Intersections") << "\n";
    ++level;
    for (int i = 0; i < spatem.intersections.list.count; ++i) {
        const IntersectionState_t* intersection = spatem.intersections.list.array[i];
        prefix("Intersection") << i << "\n";
        ++level;
        prefix("ID") << intersection->id.id << "\n";
        prefix("Revision") << intersection->revision << "\n";

        // Status bits
        prefix("Status") << "\n";
        ++level;
        for (size_t b = 0; b < intersection->status.size; ++b) {
            prefix(("Byte " + std::to_string(b)).c_str()) << std::bitset<8>(intersection->status.buf[b]) << "\n";
        }
        --level;

        // Movements
        prefix("Movements") << "\n";
        ++level;
        for (int j = 0; j < intersection->states.list.count; ++j) {
            const MovementState_t* movement = intersection->states.list.array[j];
            prefix("MovementState") << j << "\n";
            ++level;
            prefix("Signal Group") << movement->signalGroup << "\n";

            // Movement events
            prefix("State Time Speed") << "\n";
            ++level;
            for (int k = 0; k < movement->state_time_speed.list.count; ++k) {
                const MovementEvent_t* event = movement->state_time_speed.list.array[k];
                ++level;
                prefix("Event State") << static_cast<int>(event->eventState) << "\n";
                --level;
            }
            --level; // end Movement Events
            --level; // end Movement
        }
        --level; // end Movements
        --level; // end Intersection
    }
    --level; // end Intersections
}

void print_indentedCPM(std::ostream& os, const asn1::Cpm& message, const std::string& indent, unsigned level)
{
    auto prefix = [&](const char* field) -> std::ostream& {
        for (unsigned i = 0; i < level; ++i) {
            os << indent;
        }
        os << field << ": ";
        return os;
    };

    // ITS PDU Header
    const ItsPduHeader_t& header = message->header;
    prefix("ITS PDU Header") << "\n";
    ++level;
    prefix("Protocol Version") << static_cast<int>(header.protocolVersion) << "\n";
    prefix("Message ID") << static_cast<int>(header.messageID) << "\n";
    prefix("Station ID") << header.stationID << "\n";
    --level;

    // CPM content
    const CollectivePerceptionMessage_t& cpm = message->cpm;
    prefix("CPM") << "\n";
    ++level;

    prefix("Generation Delta Time") << cpm.generationDeltaTime << "\n";

    // CPM Parameters
    prefix("CPM Parameters") << "\n";
    ++level;

    // Management Container
    prefix("Management Container") << "\n";
    ++level;
    const CpmManagementContainer_t& mgmt = cpm.cpmParameters.managementContainer;

    prefix("Station Type") << static_cast<int>(mgmt.stationType) << "\n";

    prefix("Reference Position") << "\n";
    ++level;
    prefix("Latitude") << mgmt.referencePosition.latitude << "\n";
    prefix("Longitude") << mgmt.referencePosition.longitude << "\n";

    prefix("Position Confidence Ellipse") << "\n";
    ++level;
    prefix("Semi Major Confidence") << mgmt.referencePosition.positionConfidenceEllipse.semiMajorConfidence << "\n";
    prefix("Semi Minor Confidence") << mgmt.referencePosition.positionConfidenceEllipse.semiMinorConfidence << "\n";
    prefix("Semi Major Orientation") << mgmt.referencePosition.positionConfidenceEllipse.semiMajorOrientation << "\n";
    --level;

    prefix("Altitude") << "\n";
    ++level;
    prefix("Altitude Value") << mgmt.referencePosition.altitude.altitudeValue << "\n";
    prefix("Altitude Confidence") << static_cast<int>(mgmt.referencePosition.altitude.altitudeConfidence) << "\n";
    --level; // end altitude
    --level; // end reference position
    --level; // end management container

    // Perceived Object Container
   prefix("Perceived Object Container") << "\n";
    ++level;

    // Iterate over perceived objects (if there are multiple)
const PerceivedObjectContainer_t& perceived_objects = *cpm.cpmParameters.perceivedObjectContainer;

for (int i = 0; i < perceived_objects.list.count; ++i) {
    prefix(("Perceived Object " + std::to_string(i+1)).c_str()) << "\n";
        ++level;
        const PerceivedObject_t& obj = *perceived_objects.list.array[i];

        prefix("Object ID") << obj.objectID << "\n";
        prefix("Time Of Measurement") << obj.timeOfMeasurement << "\n";
        prefix("Object Confidence") << static_cast<int>(obj.objectConfidence) << "\n";

        prefix("X Distance") << "\n";
        ++level;
        prefix("Value") << obj.xDistance.value << "\n";
        prefix("Confidence") << static_cast<int>(obj.xDistance.confidence) << "\n";
        --level;

        prefix("Y Distance") << "\n";
        ++level;
        prefix("Value") << obj.yDistance.value << "\n";
        prefix("Confidence") << static_cast<int>(obj.yDistance.confidence) << "\n";
        --level;

        prefix("Z Distance") << "\n";
        ++level;
        prefix("Value") << obj.zDistance->value << "\n";
        prefix("Confidence") << static_cast<int>(obj.zDistance->confidence) << "\n";
        --level;

        prefix("X Speed") << "\n";
        ++level;
        prefix("Value") << obj.xSpeed.value << "\n";
        prefix("Confidence") << static_cast<int>(obj.xSpeed.confidence) << "\n";
        --level;

        prefix("Y Speed") << "\n";
        ++level;
        prefix("Value") << obj.ySpeed.value << "\n";
        prefix("Confidence") << static_cast<int>(obj.ySpeed.confidence) << "\n";
        --level;

        prefix("X Acceleration") << "\n";
        ++level;
        prefix("Longitudinal Acceleration Value") << obj.xAcceleration->longitudinalAccelerationValue << "\n";
        prefix("Longitudinal Acceleration Confidence") << static_cast<int>(obj.xAcceleration->longitudinalAccelerationConfidence) << "\n";
        --level;

        prefix("Yaw Angle") << "\n";
        ++level;
        prefix("Value") << obj.yawAngle->value << "\n";
        prefix("Confidence") << static_cast<int>(obj.yawAngle->confidence) << "\n";
        --level;

        prefix("Planar Object Dimension 1") << "\n";
        ++level;
        prefix("Value") << obj.planarObjectDimension1->value << "\n";
        prefix("Confidence") << static_cast<int>(obj.planarObjectDimension1->confidence) << "\n";
        --level;

        prefix("Vertical Object Dimension") << "\n";
        ++level;
        prefix("Value") << obj.verticalObjectDimension->value << "\n";
        
        prefix("Confidence") << static_cast<int>(obj.verticalObjectDimension->confidence) << "\n";
        --level;
        prefix("objectRefPoint") << obj.objectRefPoint << "\n";
}
    --level; // end perceived object container
    prefix("Number Of Perceived Objects") << cpm.cpmParameters.numberOfPerceivedObjects << "\n";

    
    --level; // end cpmParameters
    --level; // end CPM
}

void print_indentedDENM(std::ostream& os, const asn1::Denm& message, const std::string& indent, unsigned level)
{
    auto prefix = [&](const char* field) -> std::ostream& {
        for (unsigned i = 0; i < level; ++i) {
            os << indent;
        }
        os << field << ": ";
        return os;
    };

    // Print ITS PDU Header
    const ItsPduHeader_t& header = message->header;
    prefix("ITS PDU Header") << "\n";
    ++level;
    prefix("Protocol Version") << static_cast<int>(header.protocolVersion) << "\n";
    prefix("Message ID") << static_cast<int>(header.messageID) << "\n";
    prefix("Station ID") << header.stationID << "\n";
    --level;

    // DENM content
   
    // Management Container
    prefix("Management Container") << "\n";
    ++level;
    const ManagementContainer_t& mgmt = message->denm.management;
    
    // ActionID
    prefix("ActionID") << "\n";
    ++level;
    prefix("Originating Station ID") << mgmt.actionID.originatingStationID << "\n";
    prefix("Sequence Number") << mgmt.actionID.sequenceNumber << "\n";
    --level;

    long detectionTimeValue = 0;
    if (asn_INTEGER2long(&mgmt.detectionTime, &detectionTimeValue) == 0) {
        prefix("Detection Time") << detectionTimeValue << "\n";
    } else {
        prefix("Detection Time") << "(invalid INTEGER)" << "\n";
    }

// Reference Time
    long referenceTimeValue = 0;
    if (asn_INTEGER2long(&mgmt.referenceTime, &referenceTimeValue) == 0) {
        prefix("Reference Time") << referenceTimeValue << "\n";
    } else {
        prefix("Reference Time") << "(invalid INTEGER)" << "\n";
    }
    // Event Position
    prefix("Event Position") << "\n";
    ++level;
    prefix("Latitude") << mgmt.eventPosition.latitude << "\n";
    prefix("Longitude") << mgmt.eventPosition.longitude << "\n";

    // Position Confidence Ellipse
    prefix("Position Confidence Ellipse") << "\n";
    ++level;
    prefix("Semi Major Confidence") << mgmt.eventPosition.positionConfidenceEllipse.semiMajorConfidence << "\n";
    prefix("Semi Minor Confidence") << mgmt.eventPosition.positionConfidenceEllipse.semiMinorConfidence << "\n";
    prefix("Semi Major Orientation") << mgmt.eventPosition.positionConfidenceEllipse.semiMajorOrientation << "\n";
    --level;

    // Altitude
    prefix("Altitude") << "\n";
    ++level;
    prefix("Altitude Value") << mgmt.eventPosition.altitude.altitudeValue << "\n";
    prefix("Altitude Confidence") << static_cast<int>(mgmt.eventPosition.altitude.altitudeConfidence) << "\n";
    --level;
    --level; // end event position
    
   if (mgmt.relevanceDistance != nullptr) {
    prefix("Relevance Distance") << static_cast<int>(*mgmt.relevanceDistance) << "\n";
} else {
    prefix("Relevance Distance") << "(null)" << "\n";
}
    prefix("Station Type") << static_cast<int>(mgmt.stationType) << "\n";
    --level; // end management container

    // Situation Container
    prefix("Situation Container") << "\n";
    ++level;
    if (message->denm.situation != nullptr) {
    const SituationContainer_t& situation = *(message->denm.situation);

    prefix("Information Quality") << static_cast<int>(situation.informationQuality) << "\n";

    // Event Type (CauseCode)
    prefix("Event Type") << "\n";
    ++level;
    prefix("Cause Code") << static_cast<int>(situation.eventType.causeCode) << "\n";
    prefix("Sub Cause Code") << static_cast<int>(situation.eventType.subCauseCode) << "\n";
    --level; // end event type

    --level; // end situation container
} else {
    prefix("Situation Container") << "(null)" << "\n";
}
}

void ITSApplication::sendCAMToServer(const std::string& data, int size) {
    std::cout << "sending to server" << std::endl;
    if (!cam_socket.is_open()) {
    std::cerr << "Socket not open!" << std::endl;
    return;
    }
    auto bufferCopy = std::make_shared<std::string>(data);  
    auto endpoint = this->cam_endpoint;
    this->cam_socket.async_send_to(
        asio::buffer(bufferCopy->data(), size),
        endpoint,
        [bufferCopy, endpoint](const std::error_code& ec, std::size_t bytes_sent) {
            if (!ec) {
                std::cout << "Sent " << bytes_sent << " bytes to "
                        << endpoint.address().to_string()
                        << ":" << endpoint.port() << std::endl;
            } else {
                std::cerr << "Send failed: " << ec.message() << std::endl;
            }
        }
    );

}
void ITSApplication::handle_receive_error(const std::error_code& error){
    std::cerr << "Receive error: " << error.message() << std::endl;
    
}
void ITSApplication::handle_message(std::size_t bytes_transferred){

    std::string data(this->recv_buffer.data(), bytes_transferred);  // Only use valid part of buffer
    try {
        // Parse JSON from received data
        nlohmann::json proto2json = nlohmann::json::parse(data);
    
        if(!proto2json["objects"].empty()){    
            this->sendCPM(proto2json["objects"]);
        }
        if(!proto2json["trafficLights"].empty()){
            this->sendSpatem(proto2json["trafficLights"]);
        }
        if (!proto2json["events"].empty()) {
            const auto& events = proto2json["events"];
        if (events.is_array()) {
            for (const auto& event : events) {
                // Create a JSON object with the single event
                nlohmann::json singleEventJson; 
                singleEventJson["events"] = event; //add event
                this->sendDenm(singleEventJson);
            }
        } else if (events.is_object()) {
            this->sendDenm(proto2json);
        } else {
            std::cerr << "events is neither array nor object" << std::endl;
        }
    }
    } catch (nlohmann::json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
    }
}

void ITSApplication::start_receive(){
    
    this->denm_socket.async_receive_from(
        asio::buffer(this->recv_buffer), this->remote_endpoint,
        [this](std::error_code error, std::size_t bytes_transferred)
        {
            if (error || bytes_transferred == 0) {
                this->handle_receive_error(error);
                return;
            }
            this->handle_message(bytes_transferred);
            start_receive();  // Keep listening
        });
}

void ITSApplication::populateStruct(char* data, Denm_Data* denm_data, int index){
     switch (index)
    {
        case 0:
            denm_data->type = atoi(data);
            break;
        case 1:
            denm_data->lat = atoi(data);
            break;
        case 2:
            denm_data->lon = atoi(data);
            break;
        default:
            break;
    }
}

int ITSApplication::createSocket(){
    
    //this->cam_socket = asio::ip::udp::socket socket(io_service);
    cam_socket.open(asio::ip::udp::v4());
   this->cam_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(this->serverIP), this->server_port);
   return 0;
}



void ITSApplication::setSendToServer(bool send_to_server){
    this->send_to_server = send_to_server;
}

void ITSApplication::setServerPort(int serverPort){
    this->server_port = serverPort;
}

void ITSApplication::setServerIP(const char * serverIP){
    this->serverIP = serverIP;
}


void ITSApplication::setStationID(int station_id){
    this->station_id = station_id;
}

void  ITSApplication::sendToServer(u_int64_t* dataToSend, int size){
    this->cam_socket.async_send_to(
            asio::buffer(dataToSend, size),
            this->cam_endpoint,
            [dataToSend](const std::error_code& ec, std::size_t bytes_sent) {
                if (!ec) {
                    std::cout << "Async sent "<< bytes_sent << " bytes)" << std::endl;
                } else {
                    std::cerr << "Send failed: " << ec.message() << std::endl;
                }
            }
    );
}

void ITSApplication::set_interval(Clock::duration interval)
{
    cam_interval_ = interval;
    runtime_.cancel(this);
    if(cam_interval_<=vanetza::Clock::duration{0}){
        std::cout << "CAM period to low, disabling" << std::endl;
        return;
    }
    
    schedule_timer();
}

void ITSApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

void ITSApplication::print_received_message(bool flag)
{
    print_rx_msg_ = flag;
}

ITSApplication::PortType ITSApplication::port()
{
    return btp::ports::CAM;
}

int decode(const asn1::Cam& recvd, char* message){
    const ItsPduHeader_t& header = recvd->header;
    const CoopAwareness_t& cam = recvd->cam;
    const BasicContainer_t& basic = cam.camParameters.basicContainer;
    const BasicVehicleContainerHighFrequency& bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    //int size = sprintf(message, "%ld;%ld;%ld;%ld;ld\n",header.stationID,basic.referencePosition.latitude,basic.referencePosition.longitude,bvc.speed.speedValue,bvc.longitudinalAcceleration.longitudinalAccelerationValue);
    int size = sprintf(
        message, 
        "{\"objectID\":%ld,\"speed\":%ld,\"speedConfidence\":%ld,\"longAcc\":%ld,\"longAccConfidence\":%ld,\"heading\":%ld,\"headingConfidence\":%ld,\"lat\":%ld,\"lon\":%ld,\"length\":%ld,\"lengthConfidence\":%ld,\"lane\":%ld,\"laneConfidence\":%ld,\"altitude\":%ld,\"altitudeConfidence\":%ld,\"vehicleLength\":%ld,\"vehicleLengthConfidence\":%ld,\"positionConfidence\":%ld}\n",
        header.stationID,
        bvc.speed.speedValue,
        bvc.speed.speedConfidence,
        bvc.longitudinalAcceleration.longitudinalAccelerationValue,
        bvc.longitudinalAcceleration.longitudinalAccelerationConfidence,
        bvc.heading.headingValue,
        bvc.heading.headingConfidence,
        basic.referencePosition.latitude,
        basic.referencePosition.longitude,
        bvc.vehicleLength.vehicleLengthValue,
        bvc.vehicleLength.vehicleLengthConfidenceIndication,
        0,
        0,
        0,
        bvc.vehicleLength.vehicleLengthValue,
        bvc.vehicleLength.vehicleLengthConfidenceIndication,
        0,
        0
        );
    return strlen(message);
}



void ITSApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    printf("Received Message\n\n");

    // Try decode as CAM
    asn1::PacketVisitor<asn1::Cam> camVisitor;
    std::shared_ptr<const asn1::Cam> cam = boost::apply_visitor(camVisitor, *packet);

    // Try decode as DENM
    asn1::PacketVisitor<asn1::Denm> denmVisitor;
    std::shared_ptr<const asn1::Denm> denm = boost::apply_visitor(denmVisitor, *packet);

    // Try decode as CPM
    asn1::PacketVisitor<asn1::Cpm> cpmVisitor;
    std::shared_ptr<const asn1::Cpm> cpm = boost::apply_visitor(cpmVisitor, *packet);

    //Try decode as SPATEM
    asn1::PacketVisitor<asn1::Spatem> spatemVisitor;
    std::shared_ptr<const asn1::Spatem> spatem = boost::apply_visitor(spatemVisitor, *packet);

    packet.get();

    if (cam) {
        std::cout << "Received CAM with decodable content" << std::endl;
        if (print_rx_msg_) {
            std::cout << "Received CAM contains\n";
            print_indented(std::cout, *cam, "  ", 1);
            
        }
        if(send_to_server){
            char message [500];
            int size = decode(*cam, message);
            json original = json::parse(message);  // your one object (not a full message)

            // Build the outgoing message
            json outgoing;

            // Add version and timestamp manually
            const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());   
            uint16_t gen_delta_time = time_now.count();
            // Copy version and timestamp
            outgoing["version"] = 1.0;
            outgoing["timestamp"] = gen_delta_time * GenerationDeltaTime_oneMilliSec;


            // Wrap the single object into an array
            outgoing["objects"] = json::array({ original });

            // Add empty events object
            outgoing["events"] = json::array();

            // Serialize and send
            std::string jsonStr = outgoing.dump();
            //std::cout << "A enviar Proto2 para a plataforma:\n" << outgoing.dump(4) << std::endl;
            this->sendCAMToServer(jsonStr, jsonStr.size());
           // this->sendToServer((u_int64_t*)message, size);
        }
    }
    else if (denm) {
        std::cout << "Received DENM with decodable content" << std::endl;
        if (print_rx_msg_) {  
            std::cout << "Received DENM contains\n";
            print_indentedDENM(std::cout, *denm, "  ", 1);
        }
		
		switch (getMultihop()) 
		{
			case MultihopType::off:
				// não fazer forwarding
				std::cout << "Multihop Off - Not Forwarding\n";
				break;
			case MultihopType::flood:
				// flooding
				std::cout << "Multihop - Flooding\n";
				break;
			case MultihopType::prob:
				// forwarding probabilístico
				std::cout << "Multihop - Prob\n";
				break;
		}
    }    
    else if (cpm) {
        
        std::cout << "Received CPM with decodable content" << std::endl;
        if (print_rx_msg_) {
            std::cout << "Received CPM contains\n";
            print_indentedCPM(std::cout, *cpm, "  ", 1);
        }
    }
    else if(spatem){
        std::cout << "Received SPATEM with decodable content" << std::endl;
        if (print_rx_msg_) {
            std::cout << "Received SPATEM contains\n";
             print_indentedSpatem(std::cout, *spatem, "  ", 1);
        }
    }
    else {
        std::cout << "Received packet with broken or unknown content" << std::endl;
    }
}

void ITSApplication::schedule_timer()
{
    runtime_.schedule(cam_interval_, std::bind(&ITSApplication::on_timer, this, std::placeholders::_1), this);
}

void ITSApplication::on_timer(Clock::time_point)
{   
   
    schedule_timer();
    vanetza::asn1::Cam message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cam;
    header.stationID = this->station_id; // some dummy value

    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    uint16_t gen_delta_time = time_now.count();

    CoopAwareness_t& cam = message->cam;
    cam.generationDeltaTime = gen_delta_time * GenerationDeltaTime_oneMilliSec;

    auto position = positioning_.position_fix();

    if (!position.confidence) {
        std::cerr << "Skipping CAM, because no good position is available, yet." << position.confidence << std::endl;
        return;
    }

    BasicContainer_t& basic = cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;
    copy(position, basic.referencePosition);
    
    cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    BasicVehicleContainerHighFrequency& bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    if( position.course.value().value() == position.course.value().value() )
    {
        bvc.heading.headingValue = position.course.value().value();
    }else{
         bvc.heading.headingValue = 0;
    }

    bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
    if(position.speed.value().value() == position.speed.value().value()){
        bvc.speed.speedValue = position.speed.value().value();
    }
    else{
        bvc.speed.speedValue = 0;
    }


    bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;

    bvc.longitudinalAcceleration.longitudinalAccelerationValue = 0;
    bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_pointOneMeterPerSecSquared;

    bvc.driveDirection = DriveDirection_forward;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;

    bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
    bvc.vehicleWidth = VehicleWidth_unavailable;

    bvc.curvature.curvatureValue = 0;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;

    bvc.yawRate.yawRateValue = YawRateValue_unavailable;

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Invalid high frequency CAM: %s" + error);
    }

    if (print_tx_msg_) {
        std::cout << "Generated CAM contains\n";
        print_indented(std::cout, message, "  ", 1);
    }

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);

    DataRequest request;
    request.its_aid = aid::CA;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("CAM application data request failed");
    }
}

void ITSApplication::sendDenm(const json& j){

   // printf("sending denm: %ld %ld %d\n", denm_data->type, denm_data->lat, denm_data->lon );
    const auto& proto2event = j["events"];
    int counter  = 1;
    vanetza::asn1::Denm message;

    // Header	
    ItsPduHeader_t& header = message->header;
	header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_denm;
    header.stationID = this->station_id;
	

    // Management
    ManagementContainer_t& management = message->denm.management;
    //action id
    //management.actionID.originatingStationID = 1;
    management.actionID.originatingStationID = atoi(proto2event.value("origin", "1").c_str());
    management.actionID.sequenceNumber = counter;


        //detection time
    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());

    //uint64_t time = 45000000000;
   /* uint64_t time = time_now.count();
    uint64_t time = 45000000000;

	// XXX: possible memory leak, free allocated memory
    management.detectionTime.buf = (uint8_t*) malloc(sizeof(uint64_t));
    management.detectionTime.size = sizeof(uint64_t);
    
    for (size_t i = 0; i < management.detectionTime.size; ++i) {
        management.detectionTime.buf[i] = (time >> (8 * (management.detectionTime.size - 1 - i))) & 0xFF;
    }

    //reference time
   // uint64_t ref_time = 45000000000;
    uint64_t ref_time = time_now.count();
    management.referenceTime.buf = (uint8_t*) malloc(sizeof(uint64_t));
    management.referenceTime.size = sizeof(uint64_t);

    for (size_t i = 0; i < management.referenceTime.size; ++i) {
        management.referenceTime.buf[i] = (ref_time >> (8 * (management.referenceTime.size - 1 - i))) & 0xFF;
    }*/

    //detectionTime and referenceTime are set using asn_long2INTEGER() — no manual malloc/free, so no leaks there.
    INTEGER_t* detectionTime = &management.detectionTime;
    long timeValue = time_now.count();
    int ret = asn_long2INTEGER(detectionTime, timeValue);
    if (ret != 0) {
        throw std::runtime_error("Failed to set detectionTime integer");
    }

    // Same for referenceTime
    INTEGER_t* referenceTime = &management.referenceTime;
    ret = asn_long2INTEGER(referenceTime, timeValue);
    if (ret != 0) {
        throw std::runtime_error("Failed to set referenceTime integer");
    }
    //pos
    /*auto pos = positioning_.position_fix();
    if (!pos.confidence) {
        std::cerr << "Skipping DENM, no valid position" << std::endl;
        return;
    }
    copy(pos, management.eventPosition);*/

    //management event position
    management.eventPosition.altitude.altitudeValue= proto2event.value("altitude", 0);
    management.eventPosition.latitude= proto2event.value("lat", 0);
    management.eventPosition.longitude= proto2event.value("lon", 0);

    //xxx possible memory leak
   management.relevanceDistance = vanetza::asn1::allocate<RelevanceDistance_t>();
    if (management.relevanceDistance == nullptr) {
        // Handle malloc failure if needed
        // e.g., log error, return, throw, etc.
    }

    // Assign the correct enum value based on radius
    int radius = proto2event.value("radius", 0);

    if (radius < 50) {
        *management.relevanceDistance = RelevanceDistance_lessThan50m;
    } else if (radius < 100) {
        *management.relevanceDistance = RelevanceDistance_lessThan100m;
    } else if (radius < 200) {
        *management.relevanceDistance = RelevanceDistance_lessThan200m;
    } else if (radius < 500) {
        *management.relevanceDistance = RelevanceDistance_lessThan500m;
    } else if (radius < 1000) {
        *management.relevanceDistance = RelevanceDistance_lessThan1000m;
    } else if (radius < 5000) {
        *management.relevanceDistance = RelevanceDistance_lessThan5km;
    } else if (radius < 10000) {
        *management.relevanceDistance = RelevanceDistance_lessThan10km;
    } else {
        *management.relevanceDistance = RelevanceDistance_over10km;
    }
    
    management.stationType = StationType_passengerCar;

    SituationContainer* situation = vanetza::asn1::allocate<SituationContainer_t>();
    const std::string eventTypeStr = proto2event.value("eventType", "unknown");
    int causeCode = 0;  // Default to "Unavailable"
    int subCauseCode = 0;  // Default to "Generic"

    if (eventTypeStr == "speeding" || eventTypeStr == "Jerk Detected") {
        causeCode = 99;  // Dangerous situation
        // subCauseCode remains 0 (generic)
    } else if (eventTypeStr == "Entity outside safe zone") {
        causeCode = 9;  // Hazardous location - Surface condition
        // subCauseCode remains 0 (generic)
    }
    // add other codes as needed

    situation->eventType.causeCode = causeCode;
    situation->eventType.subCauseCode = 0;
    message->denm.situation = situation;

    //print generated DENM
    if (print_tx_msg_) {
        std::cout << "Generated DENM contains\n";
        asn_fprint(stdout, &asn_DEF_DENM,message.operator->());
    }
    std::string error;
	if (!message.validate(error)) {
		throw std::runtime_error("Invalid DENM: " + error);
	}
    
    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);
    DataRequest request;
    request.its_aid = aid::DEN;
	
	if (this->multihopType == MultihopType::geo) 
	{
		std::cout << "Sending DEMN using GeoBroadcasting" << std::endl;
		// ... código específico para geo
	}
	else
	{
		std::cout << "Sending DEMN using Single Hop Broacasting" << std::endl;
		request.transport_type = geonet::TransportType::SHB;
	}
	
	request.communication_profile = geonet::CommunicationProfile::ITS_G5;
    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("DENM application data request failed");
    }

	
}

void ITSApplication::setMultihop(const std::string& type) 
{
	std::cout << "multihop type:" << type << std::endl;

	if (type == "off") {
		this->multihopType = MultihopType::off;
	} else if (type == "geo") {
		this->multihopType = MultihopType::geo;
	} else if (type == "flood") {
		this->multihopType = MultihopType::flood;
	} else if (type == "prob") {
		this->multihopType = MultihopType::prob;
	} else {
		std::cerr << "Invalid multihop mode - turning off " << type << std::endl;
		this->multihopType = MultihopType::off;
	}
}

MultihopType ITSApplication::getMultihop() 
{
	return multihopType;
}
void ITSApplication::sendCPM(const json& j){
    vanetza::asn1::Cpm cpmmessage;
    ItsPduHeader_t& header = cpmmessage->header;
    header.protocolVersion = 2;
    header.messageID = 14; //header for cpm 
    header.stationID = this->station_id;

    CollectivePerceptionMessage_t& cpm = cpmmessage->cpm;

    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    uint16_t gen_delta_time = time_now.count();

    cpm.generationDeltaTime = gen_delta_time * GenerationDeltaTime_oneMilliSec;
    CpmParameters_t& cpmparams = cpm.cpmParameters;
    //managementContainer
    CpmManagementContainer_t& management = cpmparams.managementContainer;
    management.stationType = StationType_passengerCar;

    //get station position
    auto position = positioning_.position_fix();
    //convert .value() cast to microdegrees
    
    double latitude_deg = position.latitude.value();
    int32_t latitude_microdeg = static_cast<int32_t>(latitude_deg * 1e7);
    
    double longitude_deg = position.longitude.value();
    int32_t longitude_microdeg = static_cast<int32_t>(longitude_deg * 1e7);
   
    //fill reference position with station position
    management.referencePosition.latitude = latitude_microdeg;
    management.referencePosition.longitude = longitude_microdeg;

    //perceivedObjectContainer
    cpmparams.perceivedObjectContainer=vanetza::asn1::allocate<PerceivedObjectContainer_t>();
    //reference positions to help calculate xDistance and yDistance
    // 1 -> 0.01 meters , 100 -> 1 meter
    //xDistance -> longitude diference in meters
    //yDistance -> latitude difernece in meters
    ReferencePosition_t& Station_latitude = *vanetza::asn1::allocate<ReferencePosition_t>();
    Station_latitude.latitude = latitude_microdeg;  
    Station_latitude.longitude = 0; 
    ReferencePosition_t& Station_longitude = *vanetza::asn1::allocate<ReferencePosition_t>();
    Station_longitude.latitude = 0;   
    Station_longitude.longitude = longitude_microdeg; 

    for (const auto& obj : j) {

        //  PerceivedObject_t* asn_obj = (PerceivedObject_t*)calloc(1, sizeof(PerceivedObject_t));
        auto asn_obj = vanetza::asn1::allocate<PerceivedObject_t>();
        
        // 1. objectID
        std::string id_str = obj["objectID"];
        try {
            // Try converting to int
            int id_int = std::stoi(id_str);
            asn_obj->objectID = id_int;
        } catch (const std::invalid_argument& e) {

            std::cerr << "Invalid objectID: not a number -> " << id_str << std::endl;
            asn_obj->objectID = 255;
        }
    
        //TimeOfMeasurement 1 -> 1ms
        asn_obj->timeOfMeasurement =0; 
       
        //yDistance 1 -> 0.01m
        //object latitude position to calculte yDistance
        ReferencePosition_t& Object_latitude = *vanetza::asn1::allocate<ReferencePosition_t>();
        Object_latitude.latitude = obj.value("lat", 0);   
        Object_latitude.longitude = 0;  
        //calculate latitude diference in meters
        units::Length d = distance(Station_latitude,Object_latitude);
        
        //verify distance if exceeds limits set as max values
        if(d.value() > 1327.67 ){    
            asn_obj->yDistance.value = 132767;
        }else if ( d.value() < -1327.68){     
            asn_obj->yDistance.value = -132768;
        }else{
            asn_obj->yDistance.value = d.value()*100;
        }
    
        asn_obj->yDistance.confidence = obj.value("positionConfidence", 0);

        //xDistance 1 -> 0.01m
        //object longitude position to calculte xDistance
        ReferencePosition_t& Object_longitude = *vanetza::asn1::allocate<ReferencePosition_t>();
        Object_longitude.latitude = 0;    
        Object_longitude.longitude = obj.value("lon", 0);  
        //calculate longitude diference in meters
        units::Length dlon = distance(Station_longitude, Object_longitude);
        //verify distance if exceeds limits set as max values
        if(dlon.value() > 1327.67 ){
            asn_obj->xDistance.value = 132767;
        }else if ( dlon.value() < -1327.68){
            asn_obj->xDistance.value = -132768;
        }else{
            asn_obj->xDistance.value = dlon.value()*100;
        }
        asn_obj->xDistance.confidence = obj.value("positionConfidence", 0);

        // zDistance - 1 -> 0.01m  
        // altitude - assuming baseline as 0
        if (obj.contains("altitude")) {
            asn_obj->zDistance = vanetza::asn1::allocate<ObjectDistanceWithConfidence_t>();
            asn_obj->zDistance->value = obj.value("altitude", 0.0); 
            asn_obj->zDistance->confidence = obj.value("altitudeConfidence", 0);
        }

        // xSpeed - 1 -> 1 cm/s 
        asn_obj->xSpeed.value = obj.value("speed", 0);
        asn_obj->xSpeed.confidence = obj.value("speedConfidence", 0);

        //ySpeed - 0
        asn_obj->ySpeed.value = 0;
        asn_obj->ySpeed.confidence = 100;


        // xAccelerarion 1 -> 0.1m/s^2 
        if (obj.contains("longAcc")) {
            asn_obj->xAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration_t>();
            asn_obj->xAcceleration->longitudinalAccelerationValue = obj.value("longAcc", 0.0);  
            asn_obj->xAcceleration->longitudinalAccelerationConfidence = obj.value("longAccConfidence", 0);
        }

        // yawAngle 1 -> 0.1º
        if (obj.contains("heading")) {
            asn_obj->yawAngle = vanetza::asn1::allocate<CartesianAngle_t>();
            asn_obj->yawAngle->value =  obj.value("heading", 3601);  
            asn_obj->yawAngle->confidence = obj.value("headingConfidence", 0);
        }

        // Length 1 -> 0.1m
        if (obj.contains("length")) {  
            asn_obj->planarObjectDimension1 = vanetza::asn1::allocate<ObjectDimension_t>();
            asn_obj->planarObjectDimension1->value =obj.value("length", 0.0); 
            asn_obj->planarObjectDimension1->confidence = obj.value("lengthConfidence", 0);
        }
        
        // Vehicle Length 1 -> 0.1m
        if (obj.contains("vehicleLength")) {         
            asn_obj->verticalObjectDimension = vanetza::asn1::allocate<ObjectDimension_t>();
            asn_obj->verticalObjectDimension->value = obj.value("vehicleLength", 0.0);
            asn_obj->verticalObjectDimension->confidence = obj.value("vehicleLengthConfidence", 0);
        }

        // 10. Object confidence - default - unknown
        asn_obj->objectConfidence = 0;

        // object RefPoint - Defualt
        asn_obj->objectRefPoint = 0;


        // 12. Add to ASN.1 sequence
        ASN_SEQUENCE_ADD(&cpmparams.perceivedObjectContainer->list, asn_obj);
    }

    //number of perceivedobjects value
    cpmparams.numberOfPerceivedObjects = cpmparams.perceivedObjectContainer->list.count;

    if (print_tx_msg_) {
        std::cout << "Generated Full CPM contains:\n";
        asn_fprint(stdout, &asn_DEF_CPM, cpmmessage.operator->());
    }
    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(cpmmessage);
    DataRequest request;
    request.its_aid = aid::CP;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;
    
    try {
    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("CPM application data request failed");
    }
    } catch(std::runtime_error& e) {
        std::cout << "-- Vanetza UPER Encoding Error --\nCheck that the message format follows ETSI spec\n" << e.what() << std::endl;        
    }
}

void ITSApplication::sendSpatem(const json& j){
    vanetza::asn1::Spatem message;
    // Header	
    ItsPduHeader_t& header = message->header;
	header.protocolVersion = 4;
    header.messageID = ItsPduHeader__messageID_spatem;
    header.stationID = this->station_id;

    SPAT_t& spatem = message->spat;
    for (const auto& intersectionObj : j) {

        spatem.intersections= *vanetza::asn1::allocate<IntersectionStateList_t>();
        auto intersectionState = vanetza::asn1::allocate<IntersectionState_t>();
        intersectionState->id.id=intersectionObj.value("intersectionId",0);
        intersectionState->revision=10; //random number for now

        intersectionState->status.buf = (uint8_t*)calloc(2, 1);
        intersectionState->status.size = 2;
        intersectionState->status.bits_unused = 0;
        intersectionState->status.buf[0] |= (1 << (7 - (IntersectionStatusObject_trafficDependentOperation % 8)));
        
        intersectionState->states = *vanetza::asn1::allocate<MovementList_t>();
        for (const auto& stateObj : intersectionObj["states"]) {
            
            MovementState_t *state = vanetza::asn1::allocate<MovementState_t>();
            state->signalGroup = stateObj.value("signalGroup",0); //TL identifier on the intersection
            state->state_time_speed = *vanetza::asn1::allocate<MovementEventList_t>();

            MovementEvent_t *event = vanetza::asn1::allocate<MovementEvent_t>();
            event->eventState = stateObj.value("state",0); 

            ASN_SEQUENCE_ADD(&state->state_time_speed.list, event);    
            ASN_SEQUENCE_ADD(&intersectionState->states.list, state);  

        }       
        ASN_SEQUENCE_ADD(&spatem.intersections.list, intersectionState);  
    }
    
    if (print_tx_msg_) {
        std::cout << "Generated Full SPATEM contains:\n";
        asn_fprint(stdout, &asn_DEF_SPATEM, message.operator->());
    }
 

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);
    DataRequest request;
    request.its_aid = aid::SA;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;
    
    try {
    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("Spatem application data request failed");
    }
    } catch(std::runtime_error& e) {
        std::cout << "-- Vanetza UPER Encoding Error --\nCheck that the message format follows ETSI spec\n" << e.what() << std::endl;
        
        
    }
}
