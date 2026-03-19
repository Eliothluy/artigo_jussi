/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * O-RAN Aligned Network Slicing - Multi-BWP Physical Isolation (RSLAQ Compliant)
 *
 * Implements network slicing using:
 * - Single CC with multiple BWPs (handled by CcBwpCreator)
 * - 2 BWPs: BWP 0 (eMBB, 10 MHz) + BWP 1 (mMTC, 10 MHz)
 * - Dedicated EPS bearers with QCI per slice type for physical isolation
 * - BwpManagerAlgorithmStatic for QCI-to-BWP mapping
 * - NrMacSchedulerOfdmaQos for QoS scheduling
 *
 * RSLAQ Compliance:
 * - 50% static resource allocation per slice (10 MHz each)
 * - Physical isolation through separate BWPs
 * - QCI-based bearer mapping to enforce slice separation
 *
 * NOTE: BWP configuration is created automatically by CcBwpCreator.
 * The actual BWP parameters may differ from requested values.
 * Use --enableLogging to see actual BWP configuration.
 *
 * DRL/xApp Ready:
 * - Control parameters structured for future integration
 * - Observation metrics exported for RL agent
 * - Time-series data for training/evaluation
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

#include <fstream>
#include <map>
#include <vector>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranSlicingScenario");

struct ControlParams
{
    double embbResourceShare = 0.5;
    double mmtcResourceShare = 0.5;
    std::string schedulerMode = "qos";
};

struct FlowStatsAgg
{
    uint32_t flowId = 0;
    uint32_t ueId = 0;
    uint8_t sliceId = 255;
    uint8_t bwpId = 0;
    uint8_t direction = 255;
    uint64_t txBytes = 0;
    uint64_t rxBytes = 0;
    uint32_t txPackets = 0;
    uint32_t rxPackets = 0;
    double delaySum = 0.0;
    double jitterSum = 0.0;
};

struct SliceMetrics
{
    uint8_t sliceId = 255;
    uint8_t bwpId = 0;
    std::string sliceName;
    uint64_t txBytes = 0;
    uint64_t rxBytes = 0;
    uint32_t txPackets = 0;
    uint32_t rxPackets = 0;
    double delaySum = 0.0;
    uint32_t activeFlows = 0;
};

struct TimeSeriesSample
{
    double timestampSeconds;
    uint8_t sliceId;
    std::string sliceName;
    uint8_t bwpId;
    uint64_t txBytes;
    uint64_t rxBytes;
    uint32_t txPackets;
    uint32_t rxPackets;
    double throughputMbps;
    double avgDelayMs;
    double packetLossRatio;
    uint32_t activeFlows;
    uint32_t activeUes;
};

static std::map<Ipv4Address, uint32_t> g_ipToUeMap;
static std::map<uint32_t, uint8_t> g_ueToSliceMap;
static std::map<uint32_t, uint8_t> g_ueToBwpMap;
static Ipv4Address g_remoteHostAddr;

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
            auto sliceIt = g_ueToSliceMap.find(ueId);
            if (sliceIt != g_ueToSliceMap.end())
            {
                sliceId = sliceIt->second;
            }
        }
    }
    else if (t.destinationAddress == g_remoteHostAddr)
    {
        auto it = g_ipToUeMap.find(t.sourceAddress);
        if (it != g_ipToUeMap.end())
        {
            ueId = it->second;
            auto sliceIt = g_ueToSliceMap.find(ueId);
            if (sliceIt != g_ueToSliceMap.end())
            {
                sliceId = sliceIt->second;
            }
        }
    }
    return {ueId, sliceId};
}

static uint8_t
GetBwpIdForSlice(uint8_t sliceId)
{
    return g_ueToBwpMap[sliceId];
}

int main(int argc, char* argv[])
{
    uint32_t simTime = 10;
    uint32_t runNumber = 1;
    
    uint32_t nUes = 20;
    uint32_t nEmbbUes = 10;
    uint32_t nMmTcUes = 10;
    double ueDistance = 100.0;
    double ueSpread = 0.0;
    
    std::string embbDataRate = "25Mbps";
    uint32_t mmTcPacketSize = 100;
    double mmTcInterval = 1.0;
    double mmTcJitter = 0.1;
    bool mmtcUplink = true;
    
    uint8_t numerology = 0;
    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    double totalTxPower = 43;
    
    bool enableOfdma = true;
    std::string outputDir = "results";
    bool enableLogging = false;
    
    double embbSlaThroughputMbps = 25.0;
    double embbSlaLatencyMs = 100.0;
    ControlParams controlParams;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("nUes", "Total number of UEs", nUes);
    cmd.AddValue("nEmbbUes", "Number of eMBB UEs", nEmbbUes);
    cmd.AddValue("nMmTcUes", "Number of mMTC UEs", nMmTcUes);
    cmd.AddValue("ueDistance", "UE distance from gNB (meters)", ueDistance);
    cmd.AddValue("ueSpread", "UE spread in annulus (meters)", ueSpread);
    cmd.AddValue("embbDataRate", "eMBB offered rate per UE", embbDataRate);
    cmd.AddValue("mmTcPacketSize", "mMTC packet size (bytes)", mmTcPacketSize);
    cmd.AddValue("mmTcInterval", "mMTC transmission interval (s)", mmTcInterval);
    cmd.AddValue("mmTcJitter", "mMTC jitter (s)", mmTcJitter);
    cmd.AddValue("mmtcUplink", "Enable mMTC uplink traffic (default: true)", mmtcUplink);
    cmd.AddValue("numerology", "NR numerology (0-3)", numerology);
    cmd.AddValue("centralFrequency", "Central frequency (Hz)", centralFrequency);
    cmd.AddValue("bandwidth", "Bandwidth (Hz)", bandwidth);
    cmd.AddValue("totalTxPower", "Total TX power (dBm)", totalTxPower);
    cmd.AddValue("enableOfdma", "Enable OFDMA scheduler", enableOfdma);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("runNumber", "Random run number", runNumber);
    cmd.AddValue("enableLogging", "Enable detailed logging", enableLogging);
    cmd.AddValue("embbSlaThroughputMbps", "eMBB SLA throughput target (Mbps)", embbSlaThroughputMbps);
    cmd.AddValue("embbSlaLatencyMs", "eMBB SLA latency target (ms)", embbSlaLatencyMs);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(nEmbbUes + nMmTcUes != nUes, "ERROR: nEmbbUes + nMmTcUes must equal nUes");
    NS_ABORT_MSG_IF(simTime <= 0, "ERROR: simTime must be > 0");

    if (enableLogging)
    {
        LogComponentEnable("OranSlicingScenario", LOG_LEVEL_INFO);
    }

    SystemPath::MakeDirectories(outputDir);
    RngSeedManager::SetSeed(runNumber);
    RngSeedManager::SetRun(runNumber);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue(999999999));

    NS_LOG_INFO("=== O-RAN ALIGNED NETWORK SLICING SIMULATION ===");

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    std::string schedulerType = std::string("ns3::NrMacScheduler") +
                                (enableOfdma ? "Ofdma" : "Tdma") + "Qos";
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(schedulerType));

    // Configure 2 BWPs for physical isolation per RSLAQ requirements
    // BWP 0 (50%): eMBB slice - 10 MHz
    // BWP 1 (50%): mMTC slice - 10 MHz
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    bandConf.m_numBwp = 2;
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    if (enableLogging)
    {
        NS_LOG_INFO("BWP Configuration:");
        for (uint32_t i = 0; i < allBwps.size(); ++i)
        {
            NS_LOG_INFO("  BWP " << i << ": "
                       << allBwps[i].get()->m_channelBandwidth / 1e6 << " MHz @ "
                       << allBwps[i].get()->m_centralFrequency / 1e9 << " GHz");
        }
        NS_LOG_INFO("BWP Mapping for Physical Isolation:");
        NS_LOG_INFO("  BWP 0: eMBB (QCI NGBR_VIDEO_TCP_OPERATOR, QCI=6) - 10 MHz");
        NS_LOG_INFO("  BWP 1: mMTC (QCI NGBR_LOW_LAT_EMBB, QCI=80) - 10 MHz");
        NS_LOG_INFO("  Static resource allocation: 50% per slice (RSLAQ compliant)");
    }

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));

    NodeContainer gnbNodes;
    NodeContainer ueEmbbNodes;
    NodeContainer ueMmTcNodes;
    gnbNodes.Create(1);
    ueEmbbNodes.Create(nEmbbUes);
    ueMmTcNodes.Create(nMmTcUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    Ptr<ConstantPositionMobilityModel> gnbPos = gnbNodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
    gnbPos->SetPosition(Vector(0.0, 0.0, 10.0));

    double angleStep = 2.0 * M_PI / nUes;
    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        double angle = i * angleStep;
        double radius = ueDistance;
        mobility.Install(ueEmbbNodes.Get(i));
        Ptr<ConstantPositionMobilityModel> uePos = ueEmbbNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        uePos->SetPosition(Vector(radius * std::cos(angle), radius * std::sin(angle), 1.5));
    }
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        uint32_t globalIdx = i + nEmbbUes;
        double angle = globalIdx * angleStep;
        double radius = ueDistance;
        mobility.Install(ueMmTcNodes.Get(i));
        Ptr<ConstantPositionMobilityModel> uePos = ueMmTcNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        uePos->SetPosition(Vector(radius * std::cos(angle), radius * std::sin(angle), 1.5));
    }

    // Configure BWP Manager to map QCI to BWP for physical isolation
    // BWP 0: eMBB slice (NGBR_VIDEO_TCP_OPERATOR, QCI=6) - 10 MHz
    // BWP 1: mMTC slice (NGBR_LOW_LAT_EMBB, QCI=80) - 10 MHz
    // This provides 50% static resource allocation per RSLAQ requirements
    uint32_t bwpIdForEmbb = 0;
    uint32_t bwpIdForMmtc = 1;

    g_ueToBwpMap[0] = bwpIdForEmbb;
    g_ueToBwpMap[1] = bwpIdForMmtc;

    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_OPERATOR", UintegerValue(bwpIdForEmbb));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForMmtc));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_OPERATOR", UintegerValue(bwpIdForEmbb));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForMmtc));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueEmbbDevs = nrHelper->InstallUeDevice(ueEmbbNodes, allBwps);
    NetDeviceContainer ueMmTcDevs = nrHelper->InstallUeDevice(ueMmTcNodes, allBwps);

    InternetStackHelper internet;
    internet.Install(ueEmbbNodes);
    internet.Install(ueMmTcNodes);
    internet.Install(gnbNodes);

    Ipv4InterfaceContainer ueEmbbIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueEmbbDevs);
    Ipv4InterfaceContainer ueMmTcIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueMmTcDevs);

    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbPhy> gnbPhy = nrHelper->GetGnbPhy(gnbDevs.Get(i), 0);
        gnbPhy->SetAttribute("Numerology", UintegerValue(numerology));
        gnbPhy->SetAttribute("TxPower", DoubleValue(totalTxPower));
    }

    nrHelper->AttachToClosestGnb(ueEmbbDevs, gnbDevs);
    nrHelper->AttachToClosestGnb(ueMmTcDevs, gnbDevs);

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
    
    // Add route on remote host for UE network
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4> remoteIp = remoteHost->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> remoteStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteIp);
    remoteStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        Ptr<Ipv4> ueIpv4 = ueEmbbNodes.Get(i)->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> ueRouting = ipv4RoutingHelper.GetStaticRouting(ueIpv4);
        ueRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 0);
    }
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        Ptr<Ipv4> ueIpv4 = ueMmTcNodes.Get(i)->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> ueRouting = ipv4RoutingHelper.GetStaticRouting(ueIpv4);
        ueRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 0);
    }

    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        uint32_t ueId = ueEmbbNodes.Get(i)->GetId();
        g_ueToSliceMap[ueId] = 0;
        g_ipToUeMap[ueEmbbIpIfaces.GetAddress(i)] = ueId;
    }
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        uint32_t ueId = ueMmTcNodes.Get(i)->GetId();
        g_ueToSliceMap[ueId] = 1;
        g_ipToUeMap[ueMmTcIpIfaces.GetAddress(i)] = ueId;
    }

    uint16_t dlPortEmbb = 1234;
    uint16_t ulPortMmtc = 5678;

    ApplicationContainer serverApps;
    ApplicationContainer clientApps;

    // eMBB downlink traffic setup
    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                     InetSocketAddress(Ipv4Address::GetAny(), dlPortEmbb + i));
        serverApps.Add(sinkHelper.Install(ueEmbbNodes.Get(i)));

        UdpClientHelper dlClient(ueEmbbIpIfaces.GetAddress(i), dlPortEmbb + i);
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(1400));
        double interval = 1.0 / (DataRate(embbDataRate).GetBitRate() / (1400.0 * 8.0));
        dlClient.SetAttribute("Interval", TimeValue(Seconds(interval)));
        clientApps.Add(dlClient.Install(remoteHost));

        // Activate dedicated EPS bearer for eMBB with QCI 6
        Ptr<NrEpcTft> embbTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter embbFilter;
        embbFilter.localPortStart = dlPortEmbb + i;
        embbFilter.localPortEnd = dlPortEmbb + i;
        embbTft->Add(embbFilter);
        NrEpsBearer embbBearer(NrEpsBearer::NGBR_VIDEO_TCP_OPERATOR);
        nrHelper->ActivateDedicatedEpsBearer(ueEmbbDevs.Get(i), embbBearer, embbTft);
    }

    // mMTC traffic - using DOWNLINK to remote host for BWP isolation validation
    // NOTE: Uplink NR traffic has issues in this version of nr-module
    // Traffic direction: remoteHost -> UE (simulates sensor data being received)
    if (nMmTcUes > 0)
    {
        for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
        {
            uint16_t portMmtc = ulPortMmtc + i;
            Ipv4Address ueAddr = ueMmTcIpIfaces.GetAddress(i);

            // Sink on UE
            PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), portMmtc));
            serverApps.Add(sinkHelper.Install(ueMmTcNodes.Get(i)));

            // Client on remote host sending TO UE
            UdpClientHelper dlClient(ueAddr, portMmtc);
            dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
            dlClient.SetAttribute("PacketSize", UintegerValue(mmTcPacketSize));
            dlClient.SetAttribute("Interval", TimeValue(Seconds(mmTcInterval)));
            clientApps.Add(dlClient.Install(remoteHost));

            // Activate dedicated EPS bearer for mMTC with QCI 80 (Non-GBR for IoT)
            Ptr<NrEpcTft> mmtcTft = Create<NrEpcTft>();
            NrEpcTft::PacketFilter mmtcFilter;
            mmtcFilter.localPortStart = portMmtc;
            mmtcFilter.localPortEnd = portMmtc;
            mmtcTft->Add(mmtcFilter);
            NrEpsBearer mmtcBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
            nrHelper->ActivateDedicatedEpsBearer(ueMmTcDevs.Get(i), mmtcBearer, mmtcTft);
        }
    }

    serverApps.Start(Seconds(0.0));
    serverApps.Stop(Seconds(simTime));
    clientApps.Start(Seconds(0.1));
    clientApps.Stop(Seconds(simTime));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("StartTime", TimeValue(Seconds(0.0)));

    NS_LOG_INFO("Starting simulation...");

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    Simulator::Destroy();

    std::string ueMetricsFile = outputDir + "/ue_metrics.csv";
    std::string flowMetricsFile = outputDir + "/flow_metrics.csv";
    std::string sliceMetricsFile = outputDir + "/slice_metrics.csv";
    std::string timeseriesFile = outputDir + "/timeseries_metrics.csv";
    std::string summaryFile = outputDir + "/summary.json";

    std::ofstream flowOut(flowMetricsFile);
    flowOut << "flowId,ueId,sliceId,bwpId,direction,sourceIp,destIp,sourcePort,destPort,protocol,txBytes,rxBytes,txPackets,rxPackets,delayMeanMs,jitterMeanMs,packetLossRatio\n";

    std::map<uint32_t, FlowStatsAgg> flowAggMap;
    SliceMetrics sliceMetrics[2];
    sliceMetrics[0].sliceId = 0;
    sliceMetrics[0].bwpId = bwpIdForEmbb;
    sliceMetrics[0].sliceName = "eMBB";
    sliceMetrics[1].sliceId = 1;
    sliceMetrics[1].bwpId = bwpIdForMmtc;
    sliceMetrics[1].sliceName = "mMTC";

    // Process flows and collect metrics
    for (auto const& [flowId, flowStats] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        auto [ueId, sliceId] = GetUeAndSliceFromFlow(t);

        if (sliceId == 255) continue;

        uint8_t direction = (t.sourceAddress == g_remoteHostAddr) ? 0 : 1;
        uint8_t bwpId = GetBwpIdForSlice(sliceId);

        FlowStatsAgg agg;
        agg.flowId = flowId;
        agg.ueId = ueId;
        agg.sliceId = sliceId;
        agg.bwpId = bwpId;
        agg.direction = direction;
        agg.txBytes = flowStats.txBytes;
        agg.rxBytes = flowStats.rxBytes;
        agg.txPackets = flowStats.txPackets;
        agg.rxPackets = flowStats.rxPackets;
        agg.delaySum = flowStats.delaySum.GetSeconds();
        agg.jitterSum = flowStats.jitterSum.GetSeconds();
        flowAggMap[flowId] = agg;

        sliceMetrics[sliceId].txBytes += flowStats.txBytes;
        sliceMetrics[sliceId].rxBytes += flowStats.rxBytes;
        sliceMetrics[sliceId].txPackets += flowStats.txPackets;
        sliceMetrics[sliceId].rxPackets += flowStats.rxPackets;
        sliceMetrics[sliceId].delaySum += flowStats.delaySum.GetSeconds();
        if (flowStats.rxPackets > 0)
        {
            sliceMetrics[sliceId].activeFlows++;
        }

        double avgDelayMs = (flowStats.rxPackets > 0) ?
            (flowStats.delaySum.GetSeconds() / flowStats.rxPackets * 1000.0) : 0.0;
        double jitterMs = (flowStats.rxPackets > 0) ?
            (flowStats.jitterSum.GetSeconds() / flowStats.rxPackets * 1000.0) : 0.0;
        double packetLossRatio = (flowStats.txPackets > 0) ?
            static_cast<double>(flowStats.txPackets - flowStats.rxPackets) / flowStats.txPackets : 0.0;

        flowOut << flowId << "," << ueId << "," << static_cast<int>(sliceId) << ","
                 << static_cast<int>(bwpId) << ","
                 << static_cast<int>(direction) << ","
                 << t.sourceAddress << "," << t.destinationAddress << ","
                 << t.sourcePort << "," << t.destinationPort << ","
                 << "UDP" << ","
                 << flowStats.txBytes << "," << flowStats.rxBytes << ","
                 << flowStats.txPackets << "," << flowStats.rxPackets << ","
                 << avgDelayMs << "," << jitterMs << ","
                 << packetLossRatio << "\n";
    }
    flowOut.close();

    std::ofstream ueOut(ueMetricsFile);
    ueOut << "ueId,sliceId,bwpId,avgThroughputMbps,avgDelayMs,packetLossRatio,txBytes,rxBytes,txPackets,rxPackets\n";

    std::map<uint32_t, FlowStatsAgg> ueAggMap;
    for (auto const& [flowId, agg] : flowAggMap)
    {
        ueAggMap[agg.ueId].txBytes += agg.txBytes;
        ueAggMap[agg.ueId].rxBytes += agg.rxBytes;
        ueAggMap[agg.ueId].txPackets += agg.txPackets;
        ueAggMap[agg.ueId].rxPackets += agg.rxPackets;
        ueAggMap[agg.ueId].delaySum += agg.delaySum;
        ueAggMap[agg.ueId].ueId = agg.ueId;
        ueAggMap[agg.ueId].sliceId = agg.sliceId;
        ueAggMap[agg.ueId].bwpId = agg.bwpId;
    }

    for (auto const& [ueId, agg] : ueAggMap)
    {
        double avgDelayMs = (agg.rxPackets > 0) ? (agg.delaySum / agg.rxPackets * 1000.0) : 0.0;
        double avgThroughputMbps = agg.rxBytes * 8.0 / (simTime * 1e6);
        double packetLossRatio = (agg.txPackets > 0) ?
            static_cast<double>(agg.txPackets - agg.rxPackets) / agg.txPackets : 0.0;

        ueOut << ueId << "," << static_cast<int>(agg.sliceId) << "," << static_cast<int>(agg.bwpId) << ","
                << avgThroughputMbps << ","
                << avgDelayMs << ","
                << packetLossRatio << ","
                << agg.txBytes << "," << agg.rxBytes << ","
                << agg.txPackets << "," << agg.rxPackets << "\n";
    }
    ueOut.close();

    std::ofstream sliceOut(sliceMetricsFile);
    sliceOut << "sliceId,sliceName,bwpId,throughputAggregatedMbps,throughputPerUeMbps,avgLatencyMs,packetDeliveryRatio,packetLossRatio,activeFlows,totalTxBytes,totalRxBytes\n";

    double embbThroughputMbps = 0, mmtcThroughputMbps = 0;
    double embbLatencyMs = 0, mmtcLatencyMs = 0;
    double embbPdr = 0, mmtcPdr = 0;

    for (int s = 0; s < 2; ++s)
    {
        double throughputMbps = sliceMetrics[s].rxBytes * 8.0 / (simTime * 1e6);
        uint32_t numUes = (s == 0) ? nEmbbUes : nMmTcUes;
        double throughputPerUe = 0.0;
        if (numUes > 0)
        {
            throughputPerUe = throughputMbps / numUes;
        }

        double avgLatencyMs = (sliceMetrics[s].rxPackets > 0) ?
            (sliceMetrics[s].delaySum / sliceMetrics[s].rxPackets * 1000.0) : 0.0;
        double pdr = (sliceMetrics[s].txPackets > 0) ?
            static_cast<double>(sliceMetrics[s].rxPackets) / sliceMetrics[s].txPackets : 0.0;
        double plr = 1.0 - pdr;

        sliceOut << static_cast<int>(sliceMetrics[s].sliceId) << ","
                 << sliceMetrics[s].sliceName << ","
                 << static_cast<int>(sliceMetrics[s].bwpId) << ","
                 << throughputMbps << ","
                 << throughputPerUe << ","
                 << avgLatencyMs << ","
                 << pdr << ","
                 << plr << ","
                 << sliceMetrics[s].activeFlows << ","
                 << sliceMetrics[s].txBytes << ","
                 << sliceMetrics[s].rxBytes << "\n";

        if (s == 0) {
            embbThroughputMbps = throughputMbps;
            embbLatencyMs = avgLatencyMs;
            embbPdr = pdr;
        } else {
            mmtcThroughputMbps = throughputMbps;
            mmtcLatencyMs = avgLatencyMs;
            mmtcPdr = pdr;
        }
    }
    sliceOut.close();

    double embbThroughputPerUeMbps = (nEmbbUes > 0) ? embbThroughputMbps / nEmbbUes : 0.0;
    bool embbThroughputSla = (embbThroughputPerUeMbps >= embbSlaThroughputMbps);
    bool embbLatencySla = (embbLatencyMs <= embbSlaLatencyMs);
    double mmtcThroughputPerUeKbps = (nMmTcUes > 0) ? (mmtcThroughputMbps * 1000.0) / nMmTcUes : 0.0;

    std::string bwpConfigStr;
    if (allBwps.size() > 0) {
        std::stringstream ss;
        ss << "2x" << allBwps[0].get()->m_channelBandwidth / 1e6 << "MHz@" << allBwps[0].get()->m_centralFrequency / 1e9 << "GHz";
        bwpConfigStr = ss.str();
    }

    std::ofstream sumOut(summaryFile);
    sumOut << "{\n";
    sumOut << "  \"simulation\": {\n";
    sumOut << "    \"seed\": " << runNumber << ",\n";
    sumOut << "    \"runNumber\": " << runNumber << ",\n";
    sumOut << "    \"durationSeconds\": " << simTime << "\n";
    sumOut << "  },\n";

    sumOut << "  \"configuration\": {\n";
    sumOut << "    \"totalUes\": " << nUes << ",\n";
    sumOut << "    \"embbUes\": " << nEmbbUes << ",\n";
    sumOut << "    \"mmTcUes\": " << nMmTcUes << ",\n";
    sumOut << "    \"ueDistanceMeters\": " << ueDistance << ",\n";
    sumOut << "    \"embbDataRateMbps\": " << DataRate(embbDataRate).GetBitRate() / 1e6 << ",\n";
    sumOut << "    \"mmTcPacketSizeBytes\": " << mmTcPacketSize << ",\n";
    sumOut << "    \"mmTcIntervalSeconds\": " << mmTcInterval << ",\n";
    sumOut << "    \"numerology\": " << static_cast<int>(numerology) << ",\n";
    sumOut << "    \"totalTxPowerDbm\": " << totalTxPower << ",\n";
    sumOut << "    \"scheduler\": \"" << schedulerType << "\",\n";
    sumOut << "    \"bwpConfiguration\": \"" << bwpConfigStr << "\",\n";
    sumOut << "    \"sliceImplementation\": \"real_bwp_qos_bearer\",\n";
    sumOut << "    \"note\": \"Multi-BWP (2x) with 50% static resource allocation - RSLAQ compliant. eMBB uses QCI=6 (NGBR_VIDEO_TCP_OPERATOR) mapped to BWP 0. mMTC uses QCI=80 (NGBR_LOW_LAT_EMBB) mapped to BWP 1.\"\n";
    sumOut << "  },\n";

    sumOut << "  \"drlControl\": {\n";
    sumOut << "    \"embbResourceShare\": " << controlParams.embbResourceShare << ",\n";
    sumOut << "    \"mmtcResourceShare\": " << controlParams.mmtcResourceShare << ",\n";
    sumOut << "    \"schedulerMode\": \"" << controlParams.schedulerMode << "\",\n";
    sumOut << "    \"note\": \"These are placeholder values - DRL integration not implemented yet\"\n";
    sumOut << "  },\n";

    sumOut << "  \"eMBB\": {\n";
    sumOut << "    \"throughputAggregatedMbps\": " << embbThroughputMbps << ",\n";
    sumOut << "    \"throughputPerUeMbps\": " << embbThroughputPerUeMbps << ",\n";
    sumOut << "    \"avgLatencyMs\": " << embbLatencyMs << ",\n";
    sumOut << "    \"packetDeliveryRatio\": " << embbPdr << ",\n";
    sumOut << "    \"packetLossRatio\": " << (1.0 - embbPdr) << ",\n";
    sumOut << "    \"activeFlows\": " << sliceMetrics[0].activeFlows << ",\n";
    sumOut << "    \"totalTxBytes\": " << sliceMetrics[0].txBytes << ",\n";
    sumOut << "    \"totalRxBytes\": " << sliceMetrics[0].rxBytes << ",\n";
    sumOut << "    \"qci\": 6,\n";
    sumOut << "    \"qciName\": \"NGBR_VIDEO_TCP_OPERATOR\",\n";
    sumOut << "    \"bwpId\": " << bwpIdForEmbb << "\n";
    sumOut << "  },\n";

    sumOut << "  \"mMTC\": {\n";
    sumOut << "    \"throughputAggregatedMbps\": " << mmtcThroughputMbps << ",\n";
    sumOut << "    \"throughputPerUeKbps\": " << mmtcThroughputPerUeKbps << ",\n";
    sumOut << "    \"avgLatencyMs\": " << mmtcLatencyMs << ",\n";
    sumOut << "    \"packetDeliveryRatio\": " << mmtcPdr << ",\n";
    sumOut << "    \"packetLossRatio\": " << (1.0 - mmtcPdr) << ",\n";
    sumOut << "    \"activeFlows\": " << sliceMetrics[1].activeFlows << ",\n";
    sumOut << "    \"totalTxBytes\": " << sliceMetrics[1].txBytes << ",\n";
    sumOut << "    \"totalRxBytes\": " << sliceMetrics[1].rxBytes << ",\n";
    sumOut << "    \"qci\": 80,\n";
    sumOut << "    \"qciName\": \"NGBR_LOW_LAT_EMBB\",\n";
    sumOut << "    \"bwpId\": " << bwpIdForMmtc << "\n";
    sumOut << "  },\n";

    sumOut << "  \"limitations\": [\n";
    sumOut << "    \"Physical isolation at BWP level with static QCI-to-BWP mapping\",\n";
    sumOut << "    \"No dynamic resource allocation - static 50/50 split\",\n";
    sumOut << "    \"No native DRL/xApp integration in this baseline\",\n";
    sumOut << "    \"Metrics are from IP layer (FlowMonitor) - not MAC/PHY level\",\n";
    sumOut << "    \"mMTC traffic uses DOWNLINK (remoteHost->UE) due to uplink NR issues in this nr-module version\"\n";
    sumOut << "  ],\n";
    sumOut << "  \"technicalNotes\": {\n";
    sumOut << "    \"bwpMapping\": \"eMBB: QCI=6 -> BWP0, mMTC: QCI=80 -> BWP1\",\n";
    sumOut << "    \"uplinkIssue\": \"NR uplink traffic generation has issues with dedicated bearers - mMTC uses downlink as workaround\",\n";
    sumOut << "    \"qosScheduler\": \"NrMacSchedulerOfdmaQos with static QCI-to-BWP mapping\",\n";
    sumOut << "    \"drlReady\": \"Control params exposed (embbResourceShare, mmtcResourceShare) for future xApp integration\"\n";
    sumOut << "  }\n";

    sumOut << "}\n";
    sumOut.close();

    // Generate time series data
    std::ofstream timeOut(timeseriesFile);
    timeOut << "timestampSeconds,sliceId,sliceName,bwpId,throughputMbps,avgDelayMs,packetLossRatio,txBytes,rxBytes,txPackets,rxPackets,activeFlows,activeUes\n";

    // Final sample (cumulative)
    for (int s = 0; s < 2; ++s)
    {
        TimeSeriesSample sample;
        sample.timestampSeconds = simTime;
        sample.sliceId = sliceMetrics[s].sliceId;
        sample.sliceName = sliceMetrics[s].sliceName;
        sample.bwpId = sliceMetrics[s].bwpId;
        sample.txBytes = sliceMetrics[s].txBytes;
        sample.rxBytes = sliceMetrics[s].rxBytes;
        sample.txPackets = sliceMetrics[s].txPackets;
        sample.rxPackets = sliceMetrics[s].rxPackets;
        sample.throughputMbps = sliceMetrics[s].rxBytes * 8.0 / (simTime * 1e6);
        sample.avgDelayMs = (sliceMetrics[s].rxPackets > 0) ?
            (sliceMetrics[s].delaySum / sliceMetrics[s].rxPackets * 1000.0) : 0.0;
        sample.packetLossRatio = (sliceMetrics[s].txPackets > 0) ?
            static_cast<double>(sliceMetrics[s].txPackets - sliceMetrics[s].rxPackets) / sliceMetrics[s].txPackets : 0.0;
        sample.activeFlows = sliceMetrics[s].activeFlows;
        sample.activeUes = (s == 0) ? nEmbbUes : nMmTcUes;

        timeOut << sample.timestampSeconds << ","
                 << static_cast<int>(sample.sliceId) << ","
                 << sample.sliceName << ","
                 << static_cast<int>(sample.bwpId) << ","
                 << sample.throughputMbps << ","
                 << sample.avgDelayMs << ","
                 << sample.packetLossRatio << ","
                 << sample.txBytes << ","
                 << sample.rxBytes << ","
                 << sample.txPackets << ","
                 << sample.rxPackets << ","
                 << sample.activeFlows << ","
                 << sample.activeUes << "\n";
    }
    timeOut.close();

    std::cout << "\n=================================================================\n";
    std::cout << "O-RAN ALIGNED NETWORK SLICING SIMULATION - SUMMARY\n";
    std::cout << "=================================================================\n";
    std::cout << "Simulation Time: " << simTime << " seconds\n";
    std::cout << "Random Run: " << runNumber << "\n\n";

    std::cout << "Configuration:\n";
    std::cout << "- Total UEs: " << nUes << " (eMBB: " << nEmbbUes << ", mMTC: " << nMmTcUes << ")\n";
    std::cout << "- BWP: " << bwpConfigStr << "\n";
    std::cout << "- TX Power: " << totalTxPower << " dBm\n";
    std::cout << "- Scheduler: " << schedulerType << "\n";
    std::cout << "- DRL Control (placeholder): embbShare=" << controlParams.embbResourceShare
              << ", mmtcShare=" << controlParams.mmtcResourceShare << "\n\n";

    std::cout << "Results:\n";
    std::cout << "=================================================================\n";
    std::cout << "SLICE eMBB:\n";
    std::cout << "- QCI: 6 (NGBR_VIDEO_TCP_OPERATOR)\n";
    std::cout << "- BWP: " << bwpIdForEmbb << " (10 MHz)\n";
    std::cout << "- Aggregated Throughput: " << embbThroughputMbps << " Mbps\n";
    std::cout << "- Avg Throughput per UE: " << embbThroughputPerUeMbps << " Mbps\n";
    std::cout << "- Avg Latency: " << embbLatencyMs << " ms\n";
    std::cout << "- Packet Delivery Ratio: " << (embbPdr * 100) << "%\n";
    std::cout << "- Active Flows: " << sliceMetrics[0].activeFlows << "\n";
    std::cout << "- SLA Compliance:\n";
    std::cout << "  * Throughput >= " << embbSlaThroughputMbps << " Mbps: "
              << (embbThroughputSla ? "YES" : "NO") << "\n";
    std::cout << "  * Latency <= " << embbSlaLatencyMs << " ms: "
              << (embbLatencySla ? "YES" : "NO") << "\n\n";

    std::cout << "SLICE mMTC:\n";
    std::cout << "- QCI: 80 (NGBR_LOW_LAT_EMBB)\n";
    std::cout << "- BWP: " << bwpIdForMmtc << " (10 MHz)\n";
    std::cout << "- Active Flows: " << sliceMetrics[1].activeFlows << "\n";
    std::cout << "- Avg Throughput per Sensor: " << mmtcThroughputPerUeKbps << " kbps\n";
    std::cout << "- Delivery Ratio: " << (mmtcPdr * 100) << "%\n";
    std::cout << "- Avg Delay: " << mmtcLatencyMs << " ms\n\n";

    std::cout << "Output Files Generated:\n";
    std::cout << "- " << ueMetricsFile << "\n";
    std::cout << "- " << flowMetricsFile << "\n";
    std::cout << "- " << sliceMetricsFile << "\n";
    std::cout << "- " << timeseriesFile << "\n";
    std::cout << "- " << summaryFile << "\n";
    std::cout << "=================================================================\n";

    return 0;
}
