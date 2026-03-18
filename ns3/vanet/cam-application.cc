#include "cam-application.h"
#include "geo-networking.h"
#include "ns3/llc-snap-header.h"
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/mobility-model.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("CamApplication");

NS_OBJECT_ENSURE_REGISTERED(CamSender);

TypeId CamSender::GetTypeId() {
  static TypeId tid = TypeId("ns3::CamSender")
                          .SetParent<Application>()
                          .SetGroupName("Applications")
                          .AddConstructor<CamSender>();
  return tid;
}

CamSender::CamSender()
    : m_socket(nullptr),
      m_vehicleId(0),
      m_interval(Seconds(1.0)),
      m_radius(1000),
      m_running(false),
      m_packetsSent(0) {
  m_jitterRng = CreateObject<UniformRandomVariable>();
}

CamSender::~CamSender() { m_socket = nullptr; }

void CamSender::SetVehicleId(const uint32_t id) { m_vehicleId = id; }

void CamSender::SetInterval(const Time& interval) { m_interval = interval; }

void CamSender::SetBroadcastRadius(const uint16_t radius) { m_radius = radius; }

void CamSender::StartApplication() {
  m_running = true;

  if (!m_socket) {
    m_socket = Socket::CreateSocket(
        GetNode(), TypeId::LookupByName("ns3::PacketSocketFactory"));

    PacketSocketAddress socketAddr;
    socketAddr.SetProtocol(PROT_NUM_GEONETWORKING);
    socketAddr.SetSingleDevice(GetNode()->GetDevice(0)->GetIfIndex());
    socketAddr.SetPhysicalAddress(Mac48Address::GetBroadcast());
    m_socket->Bind(socketAddr);
  }

  ScheduleNextCam();
}

void CamSender::StopApplication() {
  m_running = false;

  if (m_sendEvent.IsPending()) {
    Simulator::Cancel(m_sendEvent);
  }

  if (m_socket) {
    m_socket->Close();
  }
}

void CamSender::SendCam() {
  NS_ASSERT(m_running);
  NS_ASSERT(m_socket);

  Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
  NS_ASSERT(mobility);

  Vector pos = mobility->GetPosition();
  Vector vel = mobility->GetVelocity();
  double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
  double heading = std::atan2(vel.y, vel.x) * 180.0 / M_PI;

  Ptr<Packet> packet = Create<Packet>();

  CamHeader camHeader;
  camHeader.SetVehicleId(m_vehicleId);
  camHeader.SetPositionX(pos.x);
  camHeader.SetPositionY(pos.y);
  camHeader.SetSpeed(speed);
  camHeader.SetHeading(heading);
  camHeader.SetTimestamp(Simulator::Now().GetMilliSeconds());
  packet->AddHeader(camHeader);

  GeoNetHeader geoHeader;
  geoHeader.SetVersion(1);
  geoHeader.SetNextHeader(PROT_NUM_CAM);
  geoHeader.SetMessageType(GeoNetHeader::GEOBROADCAST);
  geoHeader.SetSourcePosition(pos.x, pos.y);
  geoHeader.SetSourceId(m_vehicleId);
  geoHeader.SetRadius(m_radius);
  geoHeader.SetLifetime(10);
  packet->AddHeader(geoHeader);

  LlcSnapHeader llc;
  llc.SetType(PROT_NUM_GEONETWORKING);
  packet->AddHeader(llc);

  PacketSocketAddress destination;
  destination.SetProtocol(0);
  destination.SetSingleDevice(GetNode()->GetDevice(0)->GetIfIndex());
  destination.SetPhysicalAddress(Mac48Address::GetBroadcast());

  m_socket->SendTo(packet, 0, destination);

  m_packetsSent++;
  // Direct cout for debugging
  std::cout << "[DEBUG] Vehicle " << m_vehicleId << " sent CAM\n";
  
  NS_LOG_INFO("Vehicle " << m_vehicleId << " sent CAM at "
                         << Simulator::Now().GetSeconds() << "s"
                         << " Position: (" << pos.x << "," << pos.y << ")"
                         << " Speed: " << speed << " Heading: " << heading);

  ScheduleNextCam();
}

void CamSender::ScheduleNextCam() {
  if (m_running) {
    Time jitter = MicroSeconds(m_jitterRng->GetInteger(0, 100000));
    Time nextTime = m_interval + jitter;

    m_sendEvent = Simulator::Schedule(nextTime, &CamSender::SendCam, this);
  }
}

NS_OBJECT_ENSURE_REGISTERED(CamReceiver);

TypeId CamReceiver::GetTypeId() {
  static TypeId tid = TypeId("ns3::CamReceiver")
                          .SetParent<Application>()
                          .SetGroupName("Applications")
                          .AddConstructor<CamReceiver>();
  return tid;
}

CamReceiver::CamReceiver() : m_socket(nullptr), m_packetsReceived(0) {}

CamReceiver::~CamReceiver() { m_socket = nullptr; }

void CamReceiver::StartApplication() {
  if (!m_socket) {
    m_socket = Socket::CreateSocket(GetNode(), TypeId::LookupByName("ns3::PacketSocketFactory"));

    PacketSocketAddress socketAddr;
    socketAddr.SetProtocol(0);
    socketAddr.SetSingleDevice(GetNode()->GetDevice(0)->GetIfIndex());
    m_socket->Bind(socketAddr);
  }

  m_socket->SetRecvCallback(MakeCallback(&CamReceiver::HandleRead, this));
}

void CamReceiver::StopApplication() {
  if (m_socket) {
    m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    m_socket->Close();
  }
}

void CamReceiver::HandleRead(Ptr<Socket> socket) {
  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from))) {
    LlcSnapHeader llc;
    packet->RemoveHeader(llc);

    if (llc.GetType() == PROT_NUM_GEONETWORKING) {
      GeoNetHeader geoHeader;
      packet->RemoveHeader(geoHeader);

      Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
      NS_ASSERT(mobility);
      Vector myPos = mobility->GetPosition();

      double dx = myPos.x - geoHeader.GetSourcePositionX();
      double dy = myPos.y - geoHeader.GetSourcePositionY();
      double distance = std::sqrt(dx * dx + dy * dy);

      if (distance <= geoHeader.GetRadius()) {
        if (geoHeader.GetNextHeader() == PROT_NUM_CAM) {
          CamHeader camHeader;
          packet->RemoveHeader(camHeader);

          NS_LOG_INFO("Node "
                      << GetNode()->GetId() << " received CAM from Vehicle "
                      << camHeader.GetVehicleId() << " at "
                      << Simulator::Now().GetSeconds() << "s" << " Position: ("
                      << camHeader.GetPositionX() << ","
                      << camHeader.GetPositionY() << ")"
                      << " Speed: " << camHeader.GetSpeed()
                      << " Heading: " << camHeader.GetHeading()
                      << " Timestamp: " << camHeader.GetTimestamp() << " ms"
                      << " Distance: " << distance << "m"
                      << " GeoNet sourceId: " << geoHeader.GetSourceId());

          if (!m_rxCallback.IsNull()) {
              std::ostringstream jsonStream;
              jsonStream << "{"
                         << "\"type\":\"v2x_message\","
                         << "\"receiver_id\":" << GetNode()->GetId() << ","
                         << "\"sender_id\":" << camHeader.GetVehicleId() << ","
                         << "\"message_content\":\"CAM\","
                         << "\"timestamp\":" << Simulator::Now().GetSeconds() << ","
                         << "\"position\":{\"x\":" << camHeader.GetPositionX() << ",\"y\":" << camHeader.GetPositionY() << "},"
                         << "\"speed\":" << camHeader.GetSpeed() << ","
                         << "\"heading\":" << camHeader.GetHeading()
                         << "}";
              m_rxCallback(jsonStream.str());
          }

          // Method 3 (方案 B): Leader 的 CAM 中检测到显著减速时（例如 accel < -3.0 m/s^2），向 Python 发 brake_warning
          // main.cc 中 SetVehicleId(i+1)，故 Leader 对应 vehicleId 1
          constexpr uint32_t kLeaderVehicleId = 1;
          constexpr double kDecelThreshold = -3.0; // 显著减速阈值

          static std::map<std::pair<uint32_t, uint32_t>, double> s_lastSpeed;
          static std::map<std::pair<uint32_t, uint32_t>, uint64_t> s_lastTs;

          uint32_t myId = GetNode()->GetId();
          uint32_t vId = camHeader.GetVehicleId();
          double curSpeed = camHeader.GetSpeed();
          uint64_t curTs = camHeader.GetTimestamp();

          if (!m_rxCallback.IsNull() && vId == kLeaderVehicleId) {
              std::pair<uint32_t, uint32_t> key = {myId, vId};
              if (s_lastTs.find(key) != s_lastTs.end()) {
                  double dt = (curTs - s_lastTs[key]) / 1000.0;
                  if (dt > 0.0) {
                      double accel = (curSpeed - s_lastSpeed[key]) / dt;
                      if (accel < kDecelThreshold || (curSpeed < 1.0 && s_lastSpeed[key] >= 1.0)) {
                          std::ostringstream bw;
                          bw << "{"
                             << "\"type\":\"brake_warning\","
                             << "\"from_vehicle_id\":" << kLeaderVehicleId << ","
                             << "\"source\":\"cam\","
                             << "\"ns3_time\":" << Simulator::Now().GetSeconds() << ","
                             << "\"accel\":" << accel << ","
                             << "\"speed\":" << curSpeed
                             << "}";
                          m_rxCallback(bw.str());
                      }
                  }
              }
              s_lastSpeed[key] = curSpeed;
              s_lastTs[key] = curTs;
          }
          
          // Direct cout for debugging
          std::cout << "[DEBUG] Node " << GetNode()->GetId() << " received CAM from Vehicle " << camHeader.GetVehicleId() << "\n";

          m_packetsReceived++;
        }
      } else {
        NS_LOG_INFO("Node "
                    << GetNode()->GetId() << " discarded packet from Vehicle "
                    << geoHeader.GetSourceId()
                    << " - outside broadcast radius (distance=" << distance
                    << "m, radius=" << geoHeader.GetRadius() << "m)");
      }
    }
  }
}

}