#pragma once

#include "ns3/object.h"
#include "ns3/oran-interface.h"
#include "ns3/nr-bearer-stats-calculator.h"
#include "ns3/nr-bearer-stats-connector.h"
#include "ns3/net-device.h"

namespace ns3
{

class E2TermHelperSlicing : public Object
{
  public:
    E2TermHelperSlicing();
    ~E2TermHelperSlicing() override = default;
    static TypeId GetTypeId();
    void InstallE2Term(Ptr<NetDevice> netDevice);

  private:
    void EnableE2PdcpTraces();
    void EnableE2RlcTraces();

    Ptr<NrBearerStatsCalculator> m_e2RlcStats;
    Ptr<NrBearerStatsCalculator> m_e2PdcpStats;
    NrBearerStatsConnector m_e2StatsConnector;
    std::string m_e2ip;
    uint16_t m_e2port;
    uint16_t m_e2localPort;
};

} // namespace ns3
