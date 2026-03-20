#include "E2-term-helper-slicing.h"
#include "ns3/E2-interface-slicing.h"
#include "ns3/kpm-function-description.h"
#include "ns3/ric-control-function-description.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/config.h"
#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("E2TermHelperSlicing");
NS_OBJECT_ENSURE_REGISTERED(E2TermHelperSlicing);

E2TermHelperSlicing::E2TermHelperSlicing()
{
    NS_LOG_FUNCTION(this);
}

TypeId
E2TermHelperSlicing::GetTypeId()
{
    static TypeId tid = TypeId("ns3::E2TermHelperSlicing")
        .SetParent<Object>()
        .AddConstructor<E2TermHelperSlicing>()
        .AddAttribute("E2TermIp", "RIC E2 termination IP address",
            StringValue("10.107.233.133"),
            MakeStringAccessor(&E2TermHelperSlicing::m_e2ip),
            MakeStringChecker())
        .AddAttribute("E2Port", "RIC E2 termination port",
            UintegerValue(36422),
            MakeUintegerAccessor(&E2TermHelperSlicing::m_e2port),
            MakeUintegerChecker<uint16_t>())
        .AddAttribute("E2LocalPort", "Local SCTP bind port",
            UintegerValue(38470),
            MakeUintegerAccessor(&E2TermHelperSlicing::m_e2localPort),
            MakeUintegerChecker<uint16_t>());
    return tid;
}

void
E2TermHelperSlicing::EnableE2PdcpTraces()
{
    m_e2PdcpStats = CreateObject<NrBearerStatsCalculator>("E2PDCP");
    m_e2PdcpStats->SetAttribute("DlPdcpOutputFilename", StringValue("DlE2PdcpStats.txt"));
    m_e2PdcpStats->SetAttribute("UlPdcpOutputFilename", StringValue("UlE2PdcpStats.txt"));
    m_e2StatsConnector.EnablePdcpStats(m_e2PdcpStats);
}

void
E2TermHelperSlicing::EnableE2RlcTraces()
{
    m_e2RlcStats = CreateObject<NrBearerStatsCalculator>("E2RLC");
    m_e2RlcStats->SetAttribute("DlRlcOutputFilename", StringValue("DlE2RlcStats.txt"));
    m_e2RlcStats->SetAttribute("UlRlcOutputFilename", StringValue("UlE2RlcStats.txt"));
    m_e2StatsConnector.EnableRlcStats(m_e2RlcStats);
}

void
E2TermHelperSlicing::InstallE2Term(Ptr<NetDevice> netDevice)
{
    NS_LOG_FUNCTION(this);

    auto e2Messages = CreateObject<E2InterfaceSlicing>(netDevice);

    auto nrGnbNetDev = DynamicCast<NrGnbNetDevice>(netDevice);
    NS_ABORT_MSG_UNLESS(nrGnbNetDev, "NetDevice must be a gNB");

    std::string plmnId = "268413";
    std::string encodedPlmnId;
    if (plmnId.length() == 6)
        encodedPlmnId = {plmnId[1], plmnId[0], plmnId[3], plmnId[2], plmnId[5], plmnId[4]};
    else if (plmnId.length() == 5)
        encodedPlmnId = {plmnId[1], plmnId[0], 'F', plmnId[2], plmnId[4], plmnId[3]};

    uint16_t cellId = nrGnbNetDev->GetCellId();
    uint16_t localPort = m_e2localPort + cellId;

    EnableE2PdcpTraces();
    EnableE2RlcTraces();
    e2Messages->SetE2PdcpStatsCalculator(m_e2PdcpStats);
    e2Messages->SetE2RlcStatsCalculator(m_e2RlcStats);

    NS_LOG_INFO("E2TermHelper: PLMN=" << plmnId << " CellId=" << cellId << " Port=" << localPort);

    auto e2Term = CreateObject<E2Termination>(m_e2ip, m_e2port, localPort,
        std::to_string(cellId), encodedPlmnId);
    netDevice->AggregateObject(e2Term);

    Simulator::Schedule(Seconds(0.2), [e2Messages]() {
        Config::ConnectWithoutContext(
            "/NodeList/*/DeviceList/*/NrGnbRrc/UeMap/*/DataRadioBearerMap/*/NrRlc/TxPDU",
            MakeCallback(&E2InterfaceSlicing::ReportTxPDU, e2Messages));
    });

    Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription>();
    e2Term->RegisterKpmCallbackToE2Sm(200, kpmFd,
        std::bind(&E2InterfaceSlicing::FunctionServiceSubscriptionCallback, e2Messages, std::placeholders::_1));

    auto ricFd = Create<RicControlFunctionDescription>();
    e2Term->RegisterSmCallbackToE2Sm(300, ricFd,
        std::bind(&E2InterfaceSlicing::ControlMessageReceivedCallback, e2Messages, std::placeholders::_1));

    Simulator::Schedule(MicroSeconds(0), &E2Termination::Start, e2Term);
    netDevice->AggregateObject(e2Messages);
}

} // namespace ns3
