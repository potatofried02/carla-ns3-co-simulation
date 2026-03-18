#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket-factory.h"
#include "ns3/random-variable-stream.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-net-device.h"

#include "cam-application.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

using namespace ns3;
using json = nlohmann::json;

NS_LOG_COMPONENT_DEFINE("CarlaFixedIdVanet");

NodeContainer vehicles;
NodeContainer g_interferers;
bool g_hasInterferers = false;
std::map<int, Vector> latestPositions;
std::map<int, Vector> latestVelocities;
std::mutex dataMutex;
std::atomic firstDataReceived(false);
bool running = true;
int g_pythonSock = -1;

struct TickMessage
{
  uint64_t seq{0};
  double carla_time{0.0};
  json vehicles; // json array
};

static std::mutex g_tickMutex;
static std::condition_variable g_tickCv;
static std::deque<TickMessage> g_tickQueue;

// Task 2: Network impairment - drop V2X feedback to Python with probability packetLoss
double g_packetLoss = 0.0;
ns3::Ptr<ns3::UniformRandomVariable> g_feedbackLossRng = nullptr;

// Method 3: send brake_warning when we see Leader (id 0) braking (state-based path, ensures timing)
static bool g_leaderBrakeWarningSent = false;
static bool g_enableStateBrakeWarning = false; // debug only: bypass 802.11p, not for official experiments

static double g_dtSec = 0.1; // lock-step time step

// V2X 技术类型：wifi = 802.11p（现有实现），nr = 5G NR（后续接入 5G-LENA）。
// 当前阶段：nr 仍复用 wifi 配置，仅作为 stub 标记，便于后续切换。
static std::string g_v2xTech = "wifi";

class BackgroundTrafficApp : public Application
{
public:
  BackgroundTrafficApp() = default;

  void Setup(Ptr<NetDevice> device, Time interval, uint32_t packetSizeBytes)
  {
    m_device = device;
    m_interval = interval;
    m_packetSizeBytes = packetSizeBytes;
  }

private:
  void StartApplication() override
  {
    if (!m_device)
      return;

    m_running = true;
    m_socket = Socket::CreateSocket(GetNode(), PacketSocketFactory::GetTypeId());

    PacketSocketAddress addr;
    addr.SetSingleDevice(m_device->GetIfIndex());
    addr.SetPhysicalAddress(Mac48Address::GetBroadcast());
    addr.SetProtocol(0x9000);
    m_socket->Connect(addr);

    SendOnce();
  }

  void StopApplication() override
  {
    m_running = false;
    if (m_sendEvent.IsPending())
    {
      Simulator::Cancel(m_sendEvent);
    }
    if (m_socket)
    {
      m_socket->Close();
      m_socket = nullptr;
    }
  }

  void ScheduleNext()
  {
    if (!m_running)
      return;
    m_sendEvent = Simulator::Schedule(m_interval, &BackgroundTrafficApp::SendOnce, this);
  }

  void SendOnce()
  {
    if (!m_running || !m_socket)
      return;

    Ptr<Packet> p = Create<Packet>(m_packetSizeBytes);
    m_socket->Send(p);
    ScheduleNext();
  }

  Ptr<NetDevice> m_device;
  Ptr<Socket> m_socket;
  EventId m_sendEvent;
  Time m_interval{Seconds(0.01)};
  uint32_t m_packetSizeBytes{300};
  bool m_running{false};
};

void InitPythonConnection() {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket creation failed");
        return;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port = htons(5557);

    // Try to connect (maybe retry a few times if Python server isn't ready immediately)
    // For now, simple connect
    if (connect(sock, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        perror("connect to python failed (5557)");
        close(sock);
        return;
    }

    g_pythonSock = sock;
    std::cout << "[INFO] Connected to Python feedback channel on port 5557\n";
}

void SendToPython(std::string jsonStr) {
    if (g_pythonSock < 0) return;

    // Never drop simulation_end so Python can always exit cleanly
    bool isEndSignal = (jsonStr.find("\"type\":\"simulation_end\"") != std::string::npos);
    if (!isEndSignal && g_packetLoss > 0 && g_feedbackLossRng) {
        if (g_feedbackLossRng->GetValue(0.0, 1.0) < g_packetLoss) {
            return;  // Drop this V2X message (packet loss simulation)
        }
    }

    std::string msg = jsonStr + "\n";
    send(g_pythonSock, msg.c_str(), msg.length(), 0);

    // Method 3: CAM path also sends brake_warning via this callback; mark sent so state path does not double-send
    if (jsonStr.find("\"type\":\"brake_warning\"") != std::string::npos)
      g_leaderBrakeWarningSent = true;
}

void ProcessJsonData(const std::string &data) {
  try {
    json root = json::parse(data);
    TickMessage tm;

    // New lock-step protocol: {"type":"tick","seq":N,"carla_time":t,"vehicles":[...]}
    if (root.is_object() && root.value("type", "") == "tick")
    {
      tm.seq = root.value("seq", 0);
      tm.carla_time = root.value("carla_time", 0.0);
      tm.vehicles = root.value("vehicles", json::array());
    }
    else if (root.is_array())
    {
      // Legacy protocol: just an array of vehicles; assign seq incrementally
      static uint64_t s_legacySeq = 0;
      tm.seq = s_legacySeq++;
      tm.carla_time = Simulator::Now().GetSeconds();
      tm.vehicles = root;
    }
    else
    {
      // Unknown message
      return;
    }

    {
      std::lock_guard<std::mutex> ql(g_tickMutex);
      g_tickQueue.push_back(std::move(tm));
    }
    g_tickCv.notify_one();

    if (!firstDataReceived) {
      firstDataReceived = true;
      std::cout << "[INFO] First data received from Carla!\n";
    }

  } catch (json::exception &e) {
    std::cerr << "JSON parse error: " << e.what() << "\n";
  }
}

void SocketServerThread() {
    sockaddr_in address{};
  int addrlen = sizeof(address);
  char buffer[8192];

  int server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    perror("socket failed");
    return;
  }

  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(5556);

  if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    perror("bind failed");
    close(server_fd);
    return;
  }

  if (listen(server_fd, 1) < 0) {
    perror("listen failed");
    close(server_fd);
    return;
  }

  std::cout << "[INFO] Waiting for Carla on port 5556...\n";
  const int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&address), reinterpret_cast<socklen_t*>(&addrlen));
  if (client_fd < 0) {
    perror("accept failed");
    close(server_fd);
    return;
  }

  std::cout << "[INFO] Carla connected.\n";

  std::string lineBuffer;
  while (running) {
    memset(buffer, 0, sizeof(buffer));
    const ssize_t bytes = recv(client_fd, buffer, sizeof(buffer) - 1, 0);

    if (bytes <= 0) {
      std::cout << "[INFO] Carla disconnected or error.\n";
      break;
    }

    buffer[bytes] = '\0';
    lineBuffer += buffer;

    size_t pos;
    while ((pos = lineBuffer.find('\n')) != std::string::npos) {
      std::string line = lineBuffer.substr(0, pos);
      lineBuffer.erase(0, pos + 1);
      if (!line.empty()) {
        ProcessJsonData(line);
      }
    }
  }

  close(client_fd);
  close(server_fd);
}

static void ApplyVehiclePositionsOnce(const json& vehicleArray) {
  std::lock_guard lock(dataMutex);

  // update mobility based on this tick's vehicleArray
  for (const auto &vehicle : vehicleArray) {
    int id = vehicle.value("id", -1);
    if (id < 0) continue;
    if (static_cast<uint32_t>(id) >= vehicles.GetN()) continue;

    const Vector pos(vehicle["position"]["x"],
                     vehicle["position"]["y"],
                     vehicle["position"]["z"]);
    const Vector vel(vehicle["velocity"]["x"], vehicle["velocity"]["y"], vehicle["velocity"]["z"]);

    latestPositions[id] = pos;
    latestVelocities[id] = vel;

    Ptr<ConstantVelocityMobilityModel> mobility =
        vehicles.Get(id)->GetObject<ConstantVelocityMobilityModel>();
    if (mobility) {
      mobility->SetPosition(pos);
      mobility->SetVelocity(vel);
    }

    const bool is_braking = vehicle.value("is_braking", false);
    if (g_enableStateBrakeWarning && id == 0 && is_braking && !g_leaderBrakeWarningSent) {
      g_leaderBrakeWarningSent = true;
      std::ostringstream bw;
      bw << "{"
         << "\"type\":\"brake_warning\","
         << "\"from_vehicle_id\":1,"
         << "\"source\":\"state\","
         << "\"ns3_time\":" << Simulator::Now().GetSeconds()
         << "}";
      SendToPython(bw.str());
      std::cout << "[INFO] [stateBrakeWarning=1] Leader brake detected via is_braking flag, sent brake_warning to Python.\n";
    }
  }

  // Keep interferers near Leader so they actually contend the channel
  if (g_hasInterferers && g_interferers.GetN() > 0) {
    auto it = latestPositions.find(0);
    if (it != latestPositions.end()) {
      const Vector leaderPos = it->second;
      for (uint32_t i = 0; i < g_interferers.GetN(); i++) {
        Ptr<ConstantVelocityMobilityModel> mob =
            g_interferers.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        if (!mob) continue;
        // Small spatial offsets around leader (meters)
        const double dx = 3.0 * (i + 1);
        const double dy = 2.0 * (i + 1);
        mob->SetPosition(Vector(leaderPos.x + dx, leaderPos.y + dy, leaderPos.z));
        mob->SetVelocity(Vector(0, 0, 0));
      }
    }
  }

}

static void TickBarrier() {}

/*
void SendSimulationEndSignal() {
  int client_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (client_fd < 0) {
    perror("socket failed");
    return;
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = inet_addr("127.0.0.1");
  address.sin_port = htons(5557);

  if (connect(client_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    perror("connect failed");
    close(client_fd);
    return;
  }

  const char* end_signal = "{\"type\":\"simulation_end\"}\n";
  if (send(client_fd, end_signal, strlen(end_signal), 0) < 0) {
    perror("send failed");
  }

  close(client_fd);
}
*/

int main(int argc, char *argv[]) {
  // Enable logging for debugging
  LogComponentEnable("CarlaFixedIdVanet", LOG_LEVEL_INFO);
  LogComponentEnable("CamApplication", LOG_LEVEL_INFO);

  uint32_t nVehicles = 2;
  double simTime = 10.0;
  double camInterval = 0.1;
  uint32_t channelRegime = 1;  // 1=Good(R1), 2=Medium(R2), 3=Bad(R3), see 第4.5步
  uint32_t nInterferers = 2;   // regime=3: background contenders (dummy nodes)
  double bgIntervalSec = 0.01; // seconds
  uint32_t bgPktSizeBytes = 300;

  CommandLine cmd;
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("camInterval", "CAM interval (s)", camInterval);
  cmd.AddValue("dtSec", "Lock-step tick size (s). Must match CARLA fixed_delta_seconds.", g_dtSec);
  cmd.AddValue("packetLoss", "Probability of dropping each V2X feedback message to Python (0=no loss, 0.5=50%%)", g_packetLoss);
  cmd.AddValue("regime", "Channel regime for 802.11p: 1=Good, 2=Medium, 3=Bad (path loss + TxPower)", channelRegime);
  cmd.AddValue("nInterferers", "regime=3: number of dummy background contenders (0-2 recommended)", nInterferers);
  cmd.AddValue("bgIntervalSec", "regime=3: background traffic interval (sec) for each interferer", bgIntervalSec);
  cmd.AddValue("bgPktSizeBytes", "regime=3: background packet size (bytes)", bgPktSizeBytes);
  cmd.AddValue("stateBrakeWarning", "Enable state-based brake_warning (bypass 802.11p). Debug only; keep 0 for experiments", g_enableStateBrakeWarning);
  cmd.AddValue("v2xTech", "V2X technology: wifi (802.11p, default) or nr (5G NR / 5G-LENA stub)", g_v2xTech);
  cmd.Parse(argc, argv);

  if (channelRegime < 1 || channelRegime > 3) {
    channelRegime = 1;
    std::cout << "[WARN] regime must be 1|2|3, using 1 (Good)\n";
  }
  if (g_v2xTech != "wifi" && g_v2xTech != "nr")
  {
    std::cout << "[WARN] v2xTech must be 'wifi' or 'nr'; using 'wifi' as default.\n";
    g_v2xTech = "wifi";
  }
  std::cout << "[INFO] Channel regime = " << channelRegime << " (1=Good, 2=Medium, 3=Bad)\n";
  std::cout << "[INFO] stateBrakeWarning = " << (g_enableStateBrakeWarning ? 1 : 0)
            << " (0=CAM path only, 1=bypass 802.11p; debug only)\n";
  std::cout << "[INFO] V2X technology = " << g_v2xTech << " (wifi=802.11p, nr=5G NR stub)\n";

  if (g_packetLoss > 0) {
    g_feedbackLossRng = CreateObject<UniformRandomVariable>();
    std::cout << "[INFO] Packet loss enabled: " << (g_packetLoss * 100) << "% of V2X feedback messages will be dropped\n";
  }

  // IMPORTANT: lock-step mode uses default discrete-event simulator (not realtime).
  std::thread serverThread(SocketServerThread);

  vehicles.Create(nVehicles);

  PacketSocketHelper packetSocketHelper;
  packetSocketHelper.Install(vehicles);

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211p);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
                               StringValue("OfdmRate6Mbps"), "ControlMode",
                               StringValue("OfdmRate6Mbps"));

  // Channel regime R1/R2/R3: propagation loss + TxPower (第4.5步)
  YansWifiChannelHelper wifiChannel;
  YansWifiPhyHelper wifiPhy;

  if (g_v2xTech == "wifi")
  {
    // 802.11p (Method 3) Regime：保持第 4.5 步中定义的 R1/R2/R3「较保守」配置
    if (channelRegime == 1) {
      wifiChannel = YansWifiChannelHelper::Default();
      std::cout << "[INFO] [wifi] Regime R1 (Good): default LogDistance, default TxPower\n";
    } else {
      wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
      if (channelRegime == 2) {
        wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
            "Exponent", DoubleValue(4.0),
            "ReferenceLoss", DoubleValue(52.0));
        std::cout << "[INFO] [wifi] Regime R2 (Medium): Exponent=4.0, RefLoss=52dB, TxPower=10dBm\n";
      } else {
        wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
            "Exponent", DoubleValue(5.0),
            "ReferenceLoss", DoubleValue(58.0));
        std::cout << "[INFO] [wifi] Regime R3 (Bad): Exponent=5.0, RefLoss=58dB, TxPower=5dBm\n";
      }
    }

    wifiPhy.SetChannel(wifiChannel.Create());
    if (channelRegime == 2) {
      wifiPhy.Set("TxPowerStart", DoubleValue(10.0));
      wifiPhy.Set("TxPowerEnd", DoubleValue(10.0));
    } else if (channelRegime == 3) {
      wifiPhy.Set("TxPowerStart", DoubleValue(5.0));
      wifiPhy.Set("TxPowerEnd", DoubleValue(5.0));
    }
  }
  else // g_v2xTech == "nr" → NR-like 配置：在同一 Regime 编号下给出更优的「5G NR‑like」信道
  {
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    if (channelRegime == 1) {
      // R1: 与 wifi R1 相近，作为对照（良好信道）
      wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
          "Exponent", DoubleValue(3.0),
          "ReferenceLoss", DoubleValue(46.7));
      std::cout << "[INFO] [nr-like] Regime R1: Exponent=3.0, RefLoss=46.7dB, TxPower=20dBm\n";
    } else if (channelRegime == 2) {
      // R2: 比 wifi R2 稍好（更小指数 / 更高功率），体现 NR 在中等工况下更稳
      wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
          "Exponent", DoubleValue(3.5),
          "ReferenceLoss", DoubleValue(50.0));
      std::cout << "[INFO] [nr-like] Regime R2: Exponent=3.5, RefLoss=50dB, TxPower=15dBm\n";
    } else { // R3
      // R3: 仍然较差，但不如 wifi R3 那么「惨」，且不再叠加 MAC 竞争
      wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
          "Exponent", DoubleValue(4.0),
          "ReferenceLoss", DoubleValue(54.0));
      std::cout << "[INFO] [nr-like] Regime R3: Exponent=4.0, RefLoss=54dB, TxPower=10dBm (no extra interferers)\n";
    }

    wifiPhy.SetChannel(wifiChannel.Create());
    if (channelRegime == 1) {
      wifiPhy.Set("TxPowerStart", DoubleValue(20.0));
      wifiPhy.Set("TxPowerEnd", DoubleValue(20.0));
    } else if (channelRegime == 2) {
      wifiPhy.Set("TxPowerStart", DoubleValue(15.0));
      wifiPhy.Set("TxPowerEnd", DoubleValue(15.0));
    } else { // R3
      wifiPhy.Set("TxPowerStart", DoubleValue(10.0));
      wifiPhy.Set("TxPowerEnd", DoubleValue(10.0));
    }
  }

  WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, vehicles);
  wifiPhy.EnablePcap("../../temp/carla-vanet", devices);

  // Regime R3: add dummy contenders (MAC competition) on the same channel
  // 仅对 802.11p (wifi) 开启；NR-like 分支下不叠加哑节点，体现「同 Regime 编号下更少 MAC 竞争」。
  if (g_v2xTech == "wifi" && channelRegime == 3 && nInterferers > 0) {
    g_hasInterferers = true;
    g_interferers.Create(nInterferers);
    packetSocketHelper.Install(g_interferers);

    NetDeviceContainer interfererDevices = wifi.Install(wifiPhy, wifiMac, g_interferers);

    MobilityHelper mob2;
    Ptr<ListPositionAllocator> pos2 = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nInterferers; i++) {
      pos2->Add(Vector(0, 0, 0));
    }
    mob2.SetPositionAllocator(pos2);
    mob2.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mob2.Install(g_interferers);
    for (uint32_t i = 0; i < nInterferers; i++) {
      Ptr<ConstantVelocityMobilityModel> mob =
          g_interferers.Get(i)->GetObject<ConstantVelocityMobilityModel>();
      if (mob) mob->SetVelocity(Vector(0, 0, 0));
    }

    std::cout << "[INFO] Regime R3: enabling MAC competition with " << nInterferers
              << " dummy interferers, interval=" << bgIntervalSec
              << "s pktSize=" << bgPktSizeBytes << "B\n";

    for (uint32_t i = 0; i < nInterferers; i++) {
      Ptr<BackgroundTrafficApp> bg = CreateObject<BackgroundTrafficApp>();
      bg->Setup(interfererDevices.Get(i), Seconds(bgIntervalSec), bgPktSizeBytes);
      g_interferers.Get(i)->AddApplication(bg);
      bg->SetStartTime(Seconds(1.2 + 0.01 * i));
      bg->SetStopTime(Seconds(simTime));
    }
  }

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
  for (uint32_t i = 0; i < nVehicles; i++) {
    positionAlloc->Add(Vector(0, 0, 0));
  }

  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(vehicles);

  for (uint32_t i = 0; i < nVehicles; i++) {
    Ptr<ConstantVelocityMobilityModel> mob =
        vehicles.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(0, 0, 0));
  }

  for (uint32_t i = 0; i < nVehicles; i++) {
    Ptr<CamSender> sender = CreateObject<CamSender>();
    sender->SetVehicleId(i + 1);
    sender->SetInterval(Seconds(camInterval));
    sender->SetBroadcastRadius(1000);
    vehicles.Get(i)->AddApplication(sender);
    sender->SetStartTime(Seconds(0.0 + 0.001 * i));
    sender->SetStopTime(Seconds(simTime));

    Ptr<CamReceiver> receiver = CreateObject<CamReceiver>();
    receiver->SetRxCallback(MakeCallback(&SendToPython));
    vehicles.Get(i)->AddApplication(receiver);
    receiver->SetStartTime(Seconds(0.0));
    receiver->SetStopTime(Seconds(simTime));
  }

  std::cout << "[INFO] Waiting for first Carla data...\n";
  
  while (!firstDataReceived) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // Connect to Python feedback port after first data received (ensures Python server is ready)
  InitPythonConnection();

  std::cout << "[INFO] Starting lock-step simulation (dt=" << g_dtSec << "s)!\n";

  // Lock-step loop: wait for each tick message, apply mobility, advance ns-3 to carla_time+dt, then ack.
  while (running && Simulator::Now().GetSeconds() < simTime) {
    TickMessage tm;
    {
      std::unique_lock<std::mutex> lk(g_tickMutex);
      g_tickCv.wait(lk, [] { return !g_tickQueue.empty() || !running; });
      if (!running) break;
      tm = std::move(g_tickQueue.front());
      g_tickQueue.pop_front();
    }

    ApplyVehiclePositionsOnce(tm.vehicles);

    const double now = Simulator::Now().GetSeconds();
    // Advance ns-3 to the CARLA time carried by this tick message.
    // Python sends tick after CARLA world.tick(), so tm.carla_time matches the state timestamp.
    double targetTime = tm.carla_time;
    if (targetTime <= now) {
      // Ensure forward progress even if duplicate/non-increasing timestamps arrive.
      targetTime = now;
    }

    // Ensure the simulator advances even if there are no events in (now, targetTime]
    const Time barrierDelay = Seconds(targetTime - now);
    Simulator::Schedule(barrierDelay, &TickBarrier);
    // Stop slightly after barrier so barrier event is executed (Run stops when next event time >= stop time)
    Simulator::Stop(barrierDelay + NanoSeconds(1));
    Simulator::Run();

    std::ostringstream ack;
    ack << "{"
        << "\"type\":\"tick_ack\","
        << "\"seq\":" << tm.seq << ","
        << "\"ns3_time\":" << Simulator::Now().GetSeconds()
        << "}";
    SendToPython(ack.str());
  }

  running = false;
  serverThread.join();
  
  // Close the persistent python connection
  if (g_pythonSock >= 0) {
      // Send end signal using the existing socket
      std::string end_signal = "{\"type\":\"simulation_end\"}";
      SendToPython(end_signal);
      close(g_pythonSock);
      g_pythonSock = -1;
  }
  
  Simulator::Destroy();

  std::cout << "[INFO] Simulation finished.\n";
  return 0;
}
