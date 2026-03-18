#ifndef CAM_APPLICATION_H
#define CAM_APPLICATION_H

#include "ns3/application.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/callback.h"

namespace ns3 {

class CamSender final : public Application {
public:
    static TypeId GetTypeId();
    CamSender();
    ~CamSender() override;

    void SetVehicleId(uint32_t id);
    void SetInterval(const Time& interval);
    void SetBroadcastRadius(uint16_t radius);

private:
    void StartApplication() override;
    void StopApplication() override;

    void SendCam();
    void ScheduleNextCam();

    Ptr<Socket> m_socket;
    uint32_t m_vehicleId;
    Time m_interval;
    uint16_t m_radius;
    EventId m_sendEvent;
    bool m_running;
    uint32_t m_packetsSent;
    Ptr<UniformRandomVariable> m_jitterRng;
};

class CamReceiver final : public Application {
public:
    static TypeId GetTypeId();
    CamReceiver();
    ~CamReceiver() override;

    void SetRxCallback(Callback<void, std::string> cb) { m_rxCallback = cb; }

private:
    void StartApplication() override;
    void StopApplication() override;

    void HandleRead(Ptr<Socket> socket);

    Ptr<Socket> m_socket;
    uint32_t m_packetsReceived;
    Callback<void, std::string> m_rxCallback;
};

}

#endif