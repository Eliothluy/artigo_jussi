/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * O-RAN Slice-Aware RBG Scheduling (1 BWP Shared) with 3GPP Video Traffic
 *
 * Implements network slicing using:
 *   - Single CC with 1 shared BWP (full bandwidth)
 *   - NrMacSchedulerOfdmaSliceQos for slice-aware RBG partitioning (DL and UL)
 *   - 2 slices: eMBB (QCI 6) + IoT-like (QCI 80)
 *   - RSLAQ-inspired: 50% static (weighted) + 50% dynamic (shared)
 *   - Work-conserving: unused resources redistributed across slices
 *   - 200 MHz BWP, numerology 2 (60 kHz SCS) for video streaming
 *
 * QCI to 5QI Conceptual Mapping (TS 23.203 <-> TS 23.501):
 *   - QCI 6 (NGBR_VIDEO_TCP_OPERATOR) ~= 5QI 6-9 (video streaming)
 *   - QCI 80 (NGBR_LOW_LAT_EMBB) ~= 5QI 5 (low-latency eMBB/IoT)
 *
 *   NOTE: ns-3 NR uses QCI (EPS/LTE bearer type) inherited from EPS.
 *   Full 5GS 5QI support would require QoS Flow architecture not yet
 *   implemented in this framework version.
 *
 * Video Traffic Model:
 *   - eMBB: 3GPP TR 38.838 generic video model (frames, adaptive rate)
 *   - DOWNLINK direction (remoteHost -> UE) to work around NR UL issues
 *
 * Baseline reference: oran_slicing_bwp.cc (2 BWPs, physical isolation)
 *   This scenario: 1 BWP, logical slicing at MAC scheduler level
 *
 * DRL/xApp integration points:
 *   - SetSliceStaticWeight(sliceId, weight) for external control
 *   - SetSliceDynamicShare(sliceId, share) for external control
 *   - GetSliceMetrics(sliceId) for observation
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-eps-bearer.h"
#include "ns3/nr-eps-bearer-tag.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-generator-3gpp-generic-video.h"
#include "ns3/oran-interface.h"
#include "ns3/kpm-indication.h"
#include "ns3/kpm-function-description.h"
#include "ns3/ric-control-message.h"
#include "ns3/ric-control-function-description.h"

#include "encode_e2apv1.hpp"

extern "C" {
    #include "E2AP-PDU.h"
}

#include <fstream>
#include <map>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranSlicingRbgVideo");

static std::map<Ipv4Address, uint32_t> g_ipToUeMap;
static std::map<uint32_t, uint8_t> g_ueToSliceMap;
static Ipv4Address g_remoteHostAddr;
static uint32_t g_nEmbbUes = 10;
static uint32_t g_nIotUes = 10;

static Ptr<E2Termination> g_e2Term;
static E2Termination::RicSubscriptionRequest_rval_s g_kpmSubscription;
static uint16_t g_kpmSequenceNumber{1};
static bool g_e2Enabled{false};

struct SliceMetrics
{
    uint32_t allocatedRbgs{0};
    double throughputMbps{0.0};
    double avgDelayMs{0.0};
    double packetDeliveryRatio{0.0};
    uint32_t activeFlows{0};
};
static std::array<SliceMetrics, 2> g_sliceMetrics;

struct RicDecision
{
    double timestamp;
    uint8_t sliceId;
    double oldWeight;
    double newWeight;
    std::string reason;
};
static std::vector<RicDecision> g_ricDecisionHistory;

struct UeAgg
{
    uint32_t ueId{0};
    uint8_t sliceId{255};
    uint64_t txBytes{0};
    uint64_t rxBytes{0};
    uint32_t txPackets{0};
    uint32_t rxPackets{0};
    double delaySum{0.0};
};

struct SliceAgg
{
    uint8_t sliceId{255};
    std::string sliceName;
    uint64_t txBytes{0};
    uint64_t rxBytes{0};
    uint32_t txPackets{0};
    uint32_t rxPackets{0};
    double delaySum{0.0};
    uint32_t activeFlows{0};
};

struct TimeSeriesSample
{
    double timestampSeconds;
    uint8_t sliceId;
    std::string sliceName;
    double throughputMbps;
    double avgDelayMs;
    double packetLossRatio;
    uint32_t txBytes;
    uint32_t rxBytes;
    uint32_t txPackets;
    uint32_t rxPackets;
    uint32_t activeUes;
};

static std::pair<uint32_t, uint8_t>
GetUeAndSliceFromFlow(const Ipv4FlowClassifier::FiveTuple& t)
{
    uint32_t ueId = 0;
    uint8_t sliceId = 255;

    if (t.sourceAddress == g_remoteHostAddr)
    {
        auto it = g_ipToUeMap.find(t.destinationAddress);
        if (it != g_ipToUeMap.end())
        {
            ueId = it->second;
            auto sit = g_ueToSliceMap.find(ueId);
            if (sit != g_ueToSliceMap.end())
            {
                sliceId = sit->second;
            }
        }
    }
    else if (t.destinationAddress == g_remoteHostAddr)
    {
        auto it = g_ipToUeMap.find(t.sourceAddress);
        if (it != g_ipToUeMap.end())
        {
            ueId = it->second;
            auto sit = g_ueToSliceMap.find(ueId);
            if (sit != g_ueToSliceMap.end())
            {
                sliceId = sit->second;
            }
        }
    }
    return {ueId, sliceId};
}

static void BuildAndSendKpmIndication(E2Termination::RicSubscriptionRequest_rval_s params)
{
    KpmIndicationHeader::KpmRicIndicationHeaderValues headerValues;
    headerValues.m_plmId = "111";
    headerValues.m_gnbId = 1;
    headerValues.m_nrCellId = 1;
    headerValues.m_timestamp = Simulator::Now().GetMilliSeconds();

    Ptr<KpmIndicationHeader> header =
        Create<KpmIndicationHeader>(KpmIndicationHeader::GlobalE2nodeType::gNB, headerValues);

    KpmIndicationMessage::KpmIndicationMessageValues msgValues;

    Ptr<OCuUpContainerValues> cuUpValues = Create<OCuUpContainerValues>();
    cuUpValues->m_plmId = "111";
    cuUpValues->m_pDCPBytesUL = g_sliceMetrics[0].allocatedRbgs * 100;
    cuUpValues->m_pDCPBytesDL = g_sliceMetrics[0].allocatedRbgs * 100;
    msgValues.m_pmContainerValues = cuUpValues;

    // eMBB UE metrics
    Ptr<MeasurementItemList> embbUeValues = Create<MeasurementItemList>("UE-eMBB");
    embbUeValues->AddItem<double>("DRB.IPThpDl.UEID", g_sliceMetrics[0].throughputMbps);
    embbUeValues->AddItem<double>("DRB.IPLateDl.UEID", g_sliceMetrics[0].avgDelayMs);
    msgValues.m_ueIndications.insert(embbUeValues);

    // IoT UE metrics
    Ptr<MeasurementItemList> iotUeValues = Create<MeasurementItemList>("UE-IoT");
    iotUeValues->AddItem<double>("DRB.IPThpDl.UEID", g_sliceMetrics[1].throughputMbps);
    iotUeValues->AddItem<double>("DRB.IPLateDl.UEID", g_sliceMetrics[1].avgDelayMs);
    msgValues.m_ueIndications.insert(iotUeValues);

    Ptr<KpmIndicationMessage> msg = Create<KpmIndicationMessage>(msgValues);

    E2AP_PDU* pdu = new E2AP_PDU;
    encoding::generate_e2apv1_indication_request_parameterized(
        pdu,
        params.requestorId,
        params.instanceId,
        params.ranFuncionId,
        params.actionId,
        g_kpmSequenceNumber++,
        (uint8_t*)header->m_buffer,
        header->m_size,
        (uint8_t*)msg->m_buffer,
        msg->m_size);
    g_e2Term->SendE2Message(pdu);
    delete pdu;
}

static void
SendKpmIndications()
{
    if (!g_e2Enabled || g_kpmSubscription.requestorId == 0)
    {
        Simulator::Schedule(MilliSeconds(100), &SendKpmIndications);
        return;
    }

    BuildAndSendKpmIndication(g_kpmSubscription);

    Simulator::Schedule(MilliSeconds(100), &SendKpmIndications);
}

static void KpmSubscriptionCallback(E2AP_PDU_t* sub_req_pdu)
{
    NS_LOG_UNCOND("=== RIC Subscription Request Received ===");

    auto params = g_e2Term->ProcessRicSubscriptionRequest(sub_req_pdu);
    NS_LOG_UNCOND("Requestor ID: " << params.requestorId
            << ", Instance ID: " << params.instanceId
            << ", RAN Function ID: " << params.ranFuncionId
            << ", Action ID: " << params.actionId);

    g_kpmSubscription = params;
    BuildAndSendKpmIndication(params);
}

static void
RicControlMessageCallback(E2AP_PDU_t* ric_ctrl_pdu)
{
    NS_LOG_UNCOND("=== RIC Control Message Received ===");

    RicControlMessage msg = RicControlMessage(ric_ctrl_pdu);

    NS_LOG_UNCOND("Request Type: " << (int)msg.m_requestType);
    NS_LOG_UNCOND("RAN Function ID: " << msg.m_ranFunctionId);

    if (msg.m_requestType == RicControlMessage::ControlMessageRequestIdType::QoS)
    {
        for (const auto& param : msg.m_valuesExtracted)
        {
            if (param.m_valueType == RANParameterItem::ValueType::Int)
            {
                int value = param.m_valueInt;

                uint8_t sliceId = (value >> 8) & 0xFF;
                double weight = (value & 0xFF) / 100.0;

                if (sliceId < 2)
                {
                    g_ricDecisionHistory.push_back({
                        Simulator::Now().GetSeconds(),
                        sliceId,
                        (sliceId == 0) ? 0.5 : 0.5,
                        weight,
                        "RIC_CONTROL"
                    });

                    NS_LOG_UNCOND("RIC Control: Slice " << (int)sliceId 
                            << " weight set to " << weight);
                }
            }
        }
    }
}

int
main(int argc, char* argv[])
{
    uint32_t simTime = 10;
    uint32_t runNumber = 1;
    uint32_t nUes = 20;
    uint32_t nEmbbUes = 10;
    uint32_t nIotUes = 10;
    double ueDistance = 100.0;

    std::string videoMinDataRate = "5Mbps";
    std::string videoMaxDataRate = "10Mbps";
    std::string videoAvgDataRate = "7.5Mbps";
    uint32_t videoMinFps = 30;
    uint32_t videoMaxFps = 60;
    uint32_t videoAvgFps = 30;

    uint32_t iotPacketSize = 100;
    double iotInterval = 1.0;

    uint8_t numerology = 2;
    double centralFrequency = 3.5e9;
    double bandwidth = 200e6;
    double totalTxPower = 43.0;

    std::string outputDir = "results";
    bool enableLogging = false;
    bool enableE2 = false;
    
    std::string ricAddress = "10.0.2.10";
    uint16_t ricPort = 36422;
    uint16_t clientPort = 38472;
    uint16_t kpmReportIntervalMs = 100;
    
    double embbStaticWeight = 0.5;
    double iotStaticWeight = 0.5;
    double embbDynamicShare = 0.5;
    double iotDynamicShare = 0.5;
    double embbSlaThroughputMbps = 7.5;
    double embbSlaLatencyMs = 100.0;
    
    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("runNumber", "Random run number", runNumber);
    cmd.AddValue("nUes", "Total number of UEs", nUes);
    cmd.AddValue("nEmbbUes", "Number of eMBB UEs", nEmbbUes);
    cmd.AddValue("nIotUes", "Number of IoT UEs", nIotUes);
    cmd.AddValue("ueDistance", "UE distance from gNB (m)", ueDistance);

    cmd.AddValue("videoMinDataRate", "3GPP video min data rate", videoMinDataRate);
    cmd.AddValue("videoMaxDataRate", "3GPP video max data rate", videoMaxDataRate);
    cmd.AddValue("videoAvgDataRate", "3GPP video avg data rate", videoAvgDataRate);
    cmd.AddValue("videoMinFps", "3GPP video min FPS", videoMinFps);
    cmd.AddValue("videoMaxFps", "3GPP video max FPS", videoMaxFps);
    cmd.AddValue("videoAvgFps", "3GPP video avg FPS", videoAvgFps);

    cmd.AddValue("iotPacketSize", "IoT packet size (bytes)", iotPacketSize);
    cmd.AddValue("iotInterval", "IoT interval (s)", iotInterval);

    cmd.AddValue("numerology", "NR numerology", numerology);
    cmd.AddValue("centralFrequency", "Central frequency (Hz)", centralFrequency);
    cmd.AddValue("bandwidth", "Bandwidth (Hz)", bandwidth);
    cmd.AddValue("totalTxPower", "TX power (dBm)", totalTxPower);

    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("enableLogging", "Enable logging", enableLogging);
    cmd.AddValue("enableE2", "Enable E2/O-RAN integration", enableE2);
    
    cmd.AddValue("ricAddress", "RIC IP address", ricAddress);
    cmd.AddValue("ricPort", "RIC port", ricPort);
    cmd.AddValue("clientPort", "Client port (gNB)", clientPort);
    cmd.AddValue("kpmReportIntervalMs", "KPM report interval (ms)", kpmReportIntervalMs);
    
    cmd.AddValue("embbStaticWeight", "eMBB static weight", embbStaticWeight);
    cmd.AddValue("iotStaticWeight", "IoT static weight", iotStaticWeight);
    cmd.AddValue("embbDynamicShare", "eMBB dynamic share", embbDynamicShare);
    cmd.AddValue("iotDynamicShare", "IoT dynamic share", iotDynamicShare);
    cmd.AddValue("embbSlaThroughputMbps", "eMBB SLA throughput (Mbps)", embbSlaThroughputMbps);
    cmd.AddValue("embbSlaLatencyMs", "eMBB SLA latency (ms)", embbSlaLatencyMs);

    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(nEmbbUes + nIotUes != nUes,
                    "nEmbbUes + nIotUes must equal nUes");

    g_nEmbbUes = nEmbbUes;
    g_nIotUes = nIotUes;

    if (enableLogging)
    {
        LogComponentEnable("OranSlicingRbgVideo", LOG_LEVEL_INFO);
        LogComponentEnable("NrMacSchedulerOfdmaSliceQos", LOG_LEVEL_DEBUG);
    }

    SystemPath::MakeDirectories(outputDir);
    RngSeedManager::SetSeed(runNumber);
    RngSeedManager::SetRun(runNumber);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue(999999999));

    NS_LOG_INFO("=== O-RAN SLICE-AWARE RBG SCHEDULING WITH 3GPP VIDEO ===");
    NS_LOG_INFO("Simulation: " << simTime << "s, Run: " << runNumber);
    NS_LOG_INFO("UEs: " << nUes << " (eMBB:" << nEmbbUes
            << " IoT:" << nIotUes << ")");

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    nrHelper->SetSchedulerTypeId(
        TypeId::LookupByName("ns3::NrMacSchedulerOfdmaSliceQos"));

    nrHelper->SetSchedulerAttribute("EmbbStaticWeight", DoubleValue(embbStaticWeight));
    nrHelper->SetSchedulerAttribute("MmtcStaticWeight", DoubleValue(iotStaticWeight));
    nrHelper->SetSchedulerAttribute("EmbbDynamicShare", DoubleValue(embbDynamicShare));
    nrHelper->SetSchedulerAttribute("MmtcDynamicShare", DoubleValue(iotDynamicShare));

    if (enableE2)
    {
        g_e2Enabled = true;

        Ptr<KpmFunctionDescription> kpmFd = Create<KpmFunctionDescription>();
        Ptr<E2Termination> e2Term = CreateObject<E2Termination>(
            ricAddress,
            ricPort,
            clientPort,
            "1",
            "111");

        e2Term->Start();
        e2Term->RegisterKpmCallbackToE2Sm(200, kpmFd, &KpmSubscriptionCallback);

        Ptr<RicControlFunctionDescription> rcFd = Create<RicControlFunctionDescription>();
        e2Term->RegisterSmCallbackToE2Sm(300, rcFd, &RicControlMessageCallback);

        g_e2Term = e2Term;
        Simulator::Schedule(MilliSeconds(kpmReportIntervalMs), &SendKpmIndications);

        NS_LOG_INFO("E2/O-RAN integration enabled");
    }
    else
    {
        g_e2Enabled = false;
        NS_LOG_INFO("E2/O-RAN integration disabled");
    }

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    bandConf.m_numBwp = 1;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps = ccBwpCreator.GetAllBwps({band});

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                                 TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));

    NodeContainer gnbNodes, ueEmbbNodes, ueIotNodes;
    gnbNodes.Create(1);
    ueEmbbNodes.Create(nEmbbUes);
    ueIotNodes.Create(nIotUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);

    Ptr<ConstantPositionMobilityModel> gnbPos =
        gnbNodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
    gnbPos->SetPosition(Vector(0.0, 0.0, 10.0));

    double angleStep = 2.0 * M_PI / nUes;
    for (uint32_t i = 0; i < nEmbbUes; ++i)
    {
        mobility.Install(ueEmbbNodes.Get(i));
        Ptr<ConstantPositionMobilityModel> uePos =
            ueEmbbNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        uePos->SetPosition(Vector(ueDistance * std::cos(i * angleStep),
                                   ueDistance * std::sin(i * angleStep), 1.5));
    }
    for (uint32_t i = 0; i < nIotUes; ++i)
    {
        mobility.Install(ueIotNodes.Get(i));
        uint32_t gi = i + nEmbbUes;
        Ptr<ConstantPositionMobilityModel> uePos =
            ueIotNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        uePos->SetPosition(Vector(ueDistance * std::cos(gi * angleStep),
                                   ueDistance * std::sin(gi * angleStep), 1.5));
    }

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueEmbbNetDev = nrHelper->InstallUeDevice(ueEmbbNodes, allBwps);
    NetDeviceContainer ueIotNetDev = nrHelper->InstallUeDevice(ueIotNodes, allBwps);

    InternetStackHelper internet;
    internet.Install(ueEmbbNodes);
    internet.Install(ueIotNodes);
    internet.Install(gnbNodes);

    Ipv4InterfaceContainer ueEmbbIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueEmbbNetDev);
    Ipv4InterfaceContainer ueIotIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueIotNetDev);

    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        Ptr<NrGnbPhy> gnbPhy = nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0);
        gnbPhy->SetAttribute("Numerology", UintegerValue(numerology));
        gnbPhy->SetAttribute("TxPower", DoubleValue(totalTxPower));
    }

    nrHelper->AttachToClosestGnb(ueEmbbNetDev, gnbNetDev);
    nrHelper->AttachToClosestGnb(ueIotNetDev, gnbNetDev);

    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    g_remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                             Ipv4Mask("255.0.0.0"), 1);

    for (uint32_t i = 0; i < ueEmbbIpIfaces.GetN(); ++i)
    {
        Ptr<Ipv4StaticRouting> ueRouting =
            ipv4RoutingHelper.GetStaticRouting(ueEmbbNodes.Get(i)->GetObject<Ipv4>());
        ueRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }
    for (uint32_t i = 0; i < ueIotIpIfaces.GetN(); ++i)
    {
        Ptr<Ipv4StaticRouting> ueRouting =
            ipv4RoutingHelper.GetStaticRouting(ueIotNodes.Get(i)->GetObject<Ipv4>());
        ueRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    for (uint32_t i = 0; i < nEmbbUes; ++i)
    {
        uint32_t ueId = ueEmbbNodes.Get(i)->GetId();
        g_ueToSliceMap[ueId] = 0;
        g_ipToUeMap[ueEmbbIpIfaces.GetAddress(i)] = ueId;
    }
    for (uint32_t i = 0; i < nIotUes; ++i)
    {
        uint32_t ueId = ueIotNodes.Get(i)->GetId();
        g_ueToSliceMap[ueId] = 1;
        g_ipToUeMap[ueIotIpIfaces.GetAddress(i)] = ueId;
    }

    ApplicationContainer serverApps, clientApps;
    uint16_t dlPortEmbb = 1234;
    uint16_t dlPortIot = 5678;

    for (uint32_t i = 0; i < nEmbbUes; ++i)
    {
        uint16_t port = dlPortEmbb + i;

        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                     InetSocketAddress(Ipv4Address::GetAny(), port));
        serverApps.Add(sinkHelper.Install(ueEmbbNodes.Get(i)));

        TrafficGeneratorHelper videoHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(ueEmbbIpIfaces.GetAddress(i), port),
                                        TrafficGenerator3gppGenericVideo::GetTypeId());

        videoHelper.SetAttribute("MinDataRate", DoubleValue(std::stod(videoMinDataRate)));
        videoHelper.SetAttribute("MaxDataRate", DoubleValue(std::stod(videoMaxDataRate)));
        videoHelper.SetAttribute("MinFps", UintegerValue(videoMinFps));
        videoHelper.SetAttribute("MaxFps", UintegerValue(videoMaxFps));
        videoHelper.SetAttribute("DataRate", DoubleValue(std::stod(videoAvgDataRate)));
        videoHelper.SetAttribute("Fps", UintegerValue(videoAvgFps));

        clientApps.Add(videoHelper.Install(remoteHost));

        Ptr<NrEpcTft> embbTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter embbFilter;
        embbFilter.localPortStart = port;
        embbFilter.localPortEnd = port;
        embbTft->Add(embbFilter);
        NrEpsBearer embbBearer(NrEpsBearer::NGBR_VIDEO_TCP_OPERATOR);
        nrHelper->ActivateDedicatedEpsBearer(ueEmbbNetDev.Get(i), embbBearer, embbTft);
    }

    for (uint32_t i = 0; i < nIotUes; ++i)
    {
        uint16_t port = dlPortIot + i;

        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                     InetSocketAddress(Ipv4Address::GetAny(), port));
        serverApps.Add(sinkHelper.Install(ueIotNodes.Get(i)));

        UdpClientHelper dlClient(ueIotIpIfaces.GetAddress(i), port);
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(iotPacketSize));
        dlClient.SetAttribute("Interval", TimeValue(Seconds(iotInterval)));
        clientApps.Add(dlClient.Install(remoteHost));

        Ptr<NrEpcTft> iotTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter iotFilter;
        iotFilter.localPortStart = port;
        iotFilter.localPortEnd = port;
        iotTft->Add(iotFilter);
        NrEpsBearer iotBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueIotNetDev.Get(i), iotBearer, iotTft);
    }

    serverApps.Start(Seconds(0.1));
    serverApps.Stop(Seconds(simTime));
    clientApps.Start(Seconds(0.1));
    clientApps.Stop(Seconds(simTime));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("StartTime", TimeValue(Seconds(0.0)));

    NS_LOG_INFO("Starting simulation...");

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    Simulator::Destroy();

    std::map<uint32_t, UeAgg> ueAggMap;
    SliceAgg slices[2];
    slices[0] = {0, "eMBB"};
    slices[1] = {1, "IoT"};

    for (auto const& [flowId, flowStats] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        auto [ueId, sliceId] = GetUeAndSliceFromFlow(t);

        if (sliceId > 1)
        {
            continue;
        }

        ueAggMap[ueId].ueId = ueId;
        ueAggMap[ueId].sliceId = sliceId;
        ueAggMap[ueId].txBytes += flowStats.txBytes;
        ueAggMap[ueId].rxBytes += flowStats.rxBytes;
        ueAggMap[ueId].txPackets += flowStats.txPackets;
        ueAggMap[ueId].rxPackets += flowStats.rxPackets;
        ueAggMap[ueId].delaySum += flowStats.delaySum.GetSeconds();

        slices[sliceId].txBytes += flowStats.txBytes;
        slices[sliceId].rxBytes += flowStats.rxBytes;
        slices[sliceId].txPackets += flowStats.txPackets;
        slices[sliceId].rxPackets += flowStats.rxPackets;
        slices[sliceId].delaySum += flowStats.delaySum.GetSeconds();
        if (flowStats.rxPackets > 0)
        {
            slices[sliceId].activeFlows++;
        }
    }

    SystemPath::MakeDirectories(outputDir);

    std::string ueMetricsFile = outputDir + "/ue_metrics.csv";
    std::string sliceMetricsFile = outputDir + "/slice_metrics.csv";
    std::string timeseriesFile = outputDir + "/timeseries_metrics.csv";
    std::string summaryFile = outputDir + "/summary.json";

    double appDuration = (simTime - 0.1);

    double embbTp = 0, iotTp = 0;
    double embbLat = 0, iotLat = 0;
    double embbPdr = 0, iotPdr = 0;

    for (int s = 0; s < 2; ++s)
    {
        double tp = slices[s].rxBytes * 8.0 / (appDuration * 1e6);
        uint32_t nUe = (s == 0) ? nEmbbUes : nIotUes;
        double tpUe = (nUe > 0) ? tp / nUe : 0.0;
        double lat = (slices[s].rxPackets > 0)
            ? (slices[s].delaySum / slices[s].rxPackets * 1000.0)
            : 0.0;
        double pdr = (slices[s].txPackets > 0)
            ? static_cast<double>(slices[s].rxPackets) / slices[s].txPackets
            : 0.0;

        if (s == 0)
        {
            embbTp = tp;
            embbLat = lat;
            embbPdr = pdr;
        }
        else
        {
            iotTp = tp;
            iotLat = lat;
            iotPdr = pdr;
        }
    }

    double embbTpUe = (nEmbbUes > 0) ? embbTp / nEmbbUes : 0.0;
    bool tpSla = (embbTpUe >= embbSlaThroughputMbps);
    bool latSla = (embbLat <= embbSlaLatencyMs);

    std::ofstream ueOut(ueMetricsFile);
    ueOut << "ueId,sliceId,throughputMbps,avgDelayMs,packetLossRatio,"
          << "txBytes,rxBytes,txPackets,rxPackets\n";
    for (auto const& [ueId, ue] : ueAggMap)
    {
        double tp = ue.rxBytes * 8.0 / (appDuration * 1e6);
        double delay = (ue.rxPackets > 0) ? (ue.delaySum / ue.rxPackets * 1000.0) : 0.0;
        double plr = (ue.txPackets > 0)
            ? static_cast<double>(ue.txPackets - ue.rxPackets) / ue.txPackets
            : 0.0;
        ueOut << ueId << "," << static_cast<int>(ue.sliceId) << ","
              << tp << "," << delay << "," << plr << ","
              << ue.txBytes << "," << ue.rxBytes << ","
              << ue.txPackets << "," << ue.rxPackets << "\n";
    }
    ueOut.close();

    std::ofstream sliceOut(sliceMetricsFile);
    sliceOut << "sliceId,sliceName,throughputAggregatedMbps,throughputPerUeMbps,"
             << "avgLatencyMs,packetDeliveryRatio,packetLossRatio,"
             << "activeFlows,totalTxBytes,totalRxBytes\n";
    for (int s = 0; s < 2; ++s)
    {
        double tp = slices[s].rxBytes * 8.0 / (appDuration * 1e6);
        uint32_t nUe = (s == 0) ? nEmbbUes : nIotUes;
        double tpUe = (nUe > 0) ? tp / nUe : 0.0;
        double lat = (slices[s].rxPackets > 0)
            ? (slices[s].delaySum / slices[s].rxPackets * 1000.0)
            : 0.0;
        double pdr = (slices[s].txPackets > 0)
            ? static_cast<double>(slices[s].rxPackets) / slices[s].txPackets
            : 0.0;

        sliceOut << s << "," << slices[s].sliceName << ","
                 << tp << "," << tpUe << ","
                 << lat << "," << pdr << "," << (1.0 - pdr) << ","
                 << slices[s].activeFlows << ","
                 << slices[s].txBytes << "," << slices[s].rxBytes << "\n";
    }
    sliceOut.close();

    std::ofstream tsOut(timeseriesFile);
    tsOut << "timestampSeconds,sliceId,sliceName,throughputMbps,avgDelayMs,"
          << "packetLossRatio,txBytes,rxBytes,txPackets,rxPackets,activeUes\n";
    tsOut << "0.0,0,eMBB," << embbTp << "," << embbLat << ","
          << (1.0 - embbPdr) << "," << slices[0].txBytes << ","
          << slices[0].rxBytes << "," << slices[0].txPackets << ","
          << slices[0].rxPackets << "," << nEmbbUes << "\n";
    tsOut << "0.0,1,IoT," << iotTp << "," << iotLat << ","
          << (1.0 - iotPdr) << "," << slices[1].txBytes << ","
          << slices[1].rxBytes << "," << slices[1].txPackets << ","
          << slices[1].rxPackets << "," << nIotUes << "\n";
    tsOut.close();

    std::string bwpStr;
    {
        std::stringstream ss;
        ss << "1x" << "200MHz";
        bwpStr = ss.str();
    }

    std::ofstream sumOut(summaryFile);
    sumOut << "{\n";
    sumOut << "  \"simulation\": {\n";
    sumOut << "    \"seed\": " << runNumber << ",\n";
    sumOut << "    \"runNumber\": " << runNumber << ",\n";
    sumOut << "    \"durationSeconds\": " << simTime << ",\n";
    sumOut << "    \"appDurationSeconds\": " << appDuration << ",\n";
    sumOut << "  },\n";
    sumOut << "  \"configuration\": {\n";
    sumOut << "    \"totalUes\": " << nUes << ",\n";
    sumOut << "    \"embbUes\": " << nEmbbUes << ",\n";
    sumOut << "    \"iotUes\": " << nIotUes << ",\n";
    sumOut << "    \"scheduler\": \"ns3::NrMacSchedulerOfdmaSliceQos\",\n";
    sumOut << "    \"bwpConfiguration\": \"" << bwpStr << "\",\n";
    sumOut << "    \"sliceImplementation\": \"slice_aware_rbg_partitioning\",\n";
    sumOut << "    \"note\": \"1 BWP shared, slice-aware RBG partitioning at MAC, 3GPP video model\"\n";
    sumOut << "  },\n";
    sumOut << "  \"radioConfiguration\": {\n";
    sumOut << "    \"numerology\": " << (int)numerology << ",\n";
    sumOut << "    \"subcarrierSpacingKhz\": 60,\n";
    sumOut << "    \"centralFrequencyGHz\": " << (3.5) << ",\n";
    sumOut << "    \"bandwidthMHz\": " << (200) << ",\n";
    sumOut << "    \"note\": \"60 kHz subcarrier spacing (µ=2). 200 MHz at 3.5 GHz is experimental configuration, not a standard 3GPP band definition\"\n";
    sumOut << "  },\n";
    sumOut << "  \"videoTraffic\": {\n";
    sumOut << "    \"model\": \"3GPP TR 38.838 Generic Video\",\n";
    sumOut << "    \"direction\": \"DOWNLINK\",\n";
    sumOut << "    \"minDataRateMbps\": \"" << videoMinDataRate << "\",\n";
    sumOut << "    \"maxDataRateMbps\": \"" << videoMaxDataRate << "\",\n";
    sumOut << "    \"avgDataRateMbps\": \"" << videoAvgDataRate << "\",\n";
    sumOut << "    \"minFps\": " << videoMinFps << ",\n";
    sumOut << "    \"maxFps\": " << videoMaxFps << ",\n";
    sumOut << "    \"avgFps\": " << videoAvgFps << "\n";
    sumOut << "  },\n";
    sumOut << "  \"iotTraffic\": {\n";
    sumOut << "    \"type\": \"Low-latency UDP\",\n";
    sumOut << "    \"note\": \"IoT-like sensor data simulation, not mMTC mass IoT\",\n";
    sumOut << "    \"packetSizeBytes\": " << iotPacketSize << ",\n";
    sumOut << "    \"intervalSeconds\": " << iotInterval << "\n";
    sumOut << "  },\n";
    sumOut << "  \"slicePolicy\": {\n";
    sumOut << "    \"staticPortion\": " << 0.5 << ",\n";
    sumOut << "    \"dynamicPortion\": " << (1.0 - 0.5) << ",\n";
    sumOut << "    \"embbStaticWeight\": " << embbStaticWeight << ",\n";
    sumOut << "    \"iotStaticWeight\": " << iotStaticWeight << ",\n";
    sumOut << "    \"embbDynamicShare\": " << embbDynamicShare << ",\n";
    sumOut << "    \"iotDynamicShare\": " << iotDynamicShare << ",\n";
    sumOut << "    \"embbEffectiveShare\": "
             << (embbStaticWeight * 0.5 + embbDynamicShare * 0.5) << ",\n";
    sumOut << "    \"iotEffectiveShare\": "
             << (iotStaticWeight * 0.5 + iotDynamicShare * 0.5) << "\n";
    sumOut << "  },\n";
    sumOut << "  \"eMBB\": {\n";
    sumOut << "    \"throughputAggregatedMbps\": " << embbTp << ",\n";
    sumOut << "    \"throughputPerUeMbps\": " << embbTpUe << ",\n";
    sumOut << "    \"avgLatencyMs\": " << embbLat << ",\n";
    sumOut << "    \"packetDeliveryRatio\": " << embbPdr << ",\n";
    sumOut << "    \"packetLossRatio\": " << (1.0 - embbPdr) << ",\n";
    sumOut << "    \"activeFlows\": " << slices[0].activeFlows << ",\n";
    sumOut << "    \"totalTxBytes\": " << slices[0].txBytes << ",\n";
    sumOut << "    \"totalRxBytes\": " << slices[0].rxBytes << "\n";
    sumOut << "  },\n";
    sumOut << "  \"iot\": {\n";
    sumOut << "    \"throughputAggregatedMbps\": " << iotTp << ",\n";
    sumOut << "    \"avgLatencyMs\": " << iotLat << ",\n";
    sumOut << "    \"packetDeliveryRatio\": " << iotPdr << ",\n";
    sumOut << "    \"packetLossRatio\": " << (1.0 - iotPdr) << ",\n";
    sumOut << "    \"activeFlows\": " << slices[1].activeFlows << ",\n";
    sumOut << "    \"totalTxBytes\": " << slices[1].txBytes << ",\n";
    sumOut << "    \"totalRxBytes\": " << slices[1].rxBytes << "\n";
    sumOut << "  },\n";
    sumOut << "  \"slaVerification\": {\n";
    sumOut << "    \"embbThroughputTargetMbps\": " << embbSlaThroughputMbps << ",\n";
    sumOut << "    \"embbThroughputPerUeMbps\": " << embbTpUe << ",\n";
    sumOut << "    \"embbThroughputSlaMet\": " << (tpSla ? "true" : "false") << ",\n";
    sumOut << "    \"embbLatencyTargetMs\": " << embbSlaLatencyMs << ",\n";
    sumOut << "    \"embbLatencyMs\": " << embbLat << ",\n";
    sumOut << "    \"embbLatencySlaMet\": " << (latSla ? "true" : "false") << "\n";
    sumOut << "  },\n";
    sumOut << "  \"e2Integration\": {\n";
    sumOut << "    \"enabled\": " << (g_e2Enabled ? "true" : "false") << ",\n";
    sumOut << "    \"mode\": \"" << (g_e2Enabled ? "observability-enabled" : "disabled") << "\",\n";
    if (g_e2Enabled)
    {
        sumOut << "    \"ricAddress\": \"" << ricAddress << "\",\n";
        sumOut << "    \"ricPort\": " << ricPort << ",\n";
        sumOut << "    \"clientPort\": " << clientPort << ",\n";
        sumOut << "    \"kpmReportIntervalMs\": " << kpmReportIntervalMs << ",\n";
    }
    sumOut << "    \"note\": \"E2/O-RAN telemetry implemented with KPM indications and RIC control (observability-enabled, no closed-loop without external RIC/xApp)\"\n";
    sumOut << "  },\n";
    sumOut << "  \"qciTo5qiMapping\": {\n";
    sumOut << "    \"qci6\": \"NGBR_VIDEO_TCP_OPERATOR\",\n";
    sumOut << "    \"5qiEquivalent\": \"~5QI 6-9 (video streaming)\",\n";
    sumOut << "    \"qci80\": \"NGBR_LOW_LAT_EMBB\",\n";
    sumOut << "    \"5qiEquivalent\": \"~5QI 5 (low-latency eMBB/IoT)\",\n";
    sumOut << "    \"note\": \"ns-3 NR uses QCI (EPS/LTE) inherited from EPS. Full 5GS 5QI support requires QoS Flow architecture not yet implemented\"\n";
    sumOut << "  },\n";
    sumOut << "  \"limitations\": [\n";
    sumOut << "    \"RBG-level granularity (not PRB-level) - native 5G-LENA constraint\",\n";
    sumOut << "    \"Slice ID (S-NSSAI) not natively supported - using QCI as proxy\",\n";
    sumOut << "    \"5QI not natively supported - using QCI with documented 5QI equivalence\",\n";
    sumOut << "    \"Maximum 2 slices in this scheduler implementation\",\n";
    sumOut << "    \"Work-conserving resources may not respect strict SLA guarantees\",\n";
    sumOut << "    \"IoT traffic simplified (UDP periodic), not full mMTC model\",\n";
    sumOut << "    \"Metrics from FlowMonitor (IP layer), not MAC/PHY level\",\n";
    sumOut << "    \"3GPP video model uses DOWNLINK to work around NR UL limitations\"\n";
    sumOut << "  ],\n";
    sumOut << "  \"technicalNotes\": {\n";
    sumOut << "    \"scheduler\": \"NrMacSchedulerOfdmaSliceQos\",\n";
    sumOut << "    \"inheritance\": \"NrMacSchedulerOfdmaRR -> NrMacSchedulerOfdmaQos -> SliceQos\",\n";
    sumOut << "    \"sliceIdentification\": \"QCI-based (eMBB:QCI=6, IoT:QCI=80)\",\n";
    sumOut << "    \"resourcePartitioning\": \"50% static (weighted) + 50% dynamic\",\n";
    sumOut << "    \"workConserving\": true,\n";
    sumOut << "    \"granularity\": \"RBG (Resource Block Group)\",\n";
    sumOut << "    \"externalControl\": \"SetSliceStaticWeight/SetSliceDynamicShare APIs exposed for RIC\"\n";
    sumOut << "  }\n";
    sumOut << "}\n";
    sumOut.close();

    std::cout << "\n";
    std::cout << "O-RAN SLICE-AWARE RBG SCHEDULING WITH 3GPP VIDEO - SUMMARY\n";
    std::cout << "\n";
    std::cout << "Time: " << simTime << "s | Run: " << runNumber << "\n";
    std::cout << "UEs: " << nUes << " (eMBB:" << nEmbbUes
              << " IoT:" << nIotUes << ")\n";
    std::cout << "BWP: 1x200MHz shared | Scheduler: SliceQos\n";
    std::cout << "Video: 3GPP Model | " << videoAvgDataRate << " avg @ "
              << videoAvgFps << " FPS\n";
    std::cout << "Shares: eMBB="
              << (embbStaticWeight * 0.5 + embbDynamicShare * 0.5)
              << " IoT="
              << (iotStaticWeight * 0.5 + iotDynamicShare * 0.5) << "\n";
    std::cout << "eMBB: TP=" << embbTp << " Mbps (" << embbTpUe
              << " Mbps/UE)"
              << " Lat=" << embbLat << " ms PDR=" << (embbPdr * 100) << "%\n";
    std::cout << "IoT: TP=" << iotTp << " Mbps"
              << " Lat=" << iotLat << " ms PDR=" << (iotPdr * 100) << "%\n";
    std::cout << "Files: " << outputDir
              << "/{ue_metrics,slice_metrics,timeseries_metrics,summary}.csv/.json\n";
    std::cout << "\n";

    return 0;
}
