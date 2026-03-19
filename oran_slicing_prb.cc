/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * O-RAN Slice-Aware PRB/RBG Scheduling (1 BWP Shared)
 *
 * Implements network slicing using:
 *   - Single CC with 1 shared BWP (full bandwidth)
 *   - NrMacSchedulerOfdmaSliceQos for slice-aware RBG partitioning
 *   - 2 slices: eMBB (QCI 6) + mMTC (QCI 80)
 *   - RSLAQ-inspired: 50% static (weighted) + 50% dynamic (shared)
 *   - Work-conserving: unused resources redistributed across slices
 *
 * Baseline reference: oran_slicing_bwp.cc (2 BWPs, physical isolation)
 * This scenario: 1 BWP, logical slicing at MAC scheduler level
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

#include <fstream>
#include <map>
#include <vector>
#include <sstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranSlicingPrb");

// Global maps for flow identification
static std::map<Ipv4Address, uint32_t> g_ipToUeMap;
static std::map<uint32_t, uint8_t> g_ueToSliceMap;
static Ipv4Address g_remoteHostAddr;
static uint32_t g_nEmbbUes = 10;
static uint32_t g_nMmTcUes = 10;

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

int main(int argc, char* argv[])
{
    // Parameters
    uint32_t simTime = 10;
    uint32_t runNumber = 1;

    uint32_t nUes = 20;
    uint32_t nEmbbUes = 10;
    uint32_t nMmTcUes = 10;
    double ueDistance = 100.0;

    std::string embbDataRate = "25Mbps";
    uint32_t mmTcPacketSize = 100;
    double mmTcInterval = 1.0;

    uint8_t numerology = 0;
    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    double totalTxPower = 43;

    std::string outputDir = "results";
    bool enableLogging = false;

    double embbStaticWeight = 0.5;
    double mmtcStaticWeight = 0.5;
    double embbDynamicShare = 0.5;
    double mmtcDynamicShare = 0.5;

    double embbSlaThroughputMbps = 25.0;
    double embbSlaLatencyMs = 100.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("runNumber", "Random run number", runNumber);
    cmd.AddValue("nUes", "Total number of UEs", nUes);
    cmd.AddValue("nEmbbUes", "Number of eMBB UEs", nEmbbUes);
    cmd.AddValue("nMmTcUes", "Number of mMTC UEs", nMmTcUes);
    cmd.AddValue("ueDistance", "UE distance from gNB (m)", ueDistance);
    cmd.AddValue("embbDataRate", "eMBB offered rate per UE", embbDataRate);
    cmd.AddValue("mmTcPacketSize", "mMTC packet size (bytes)", mmTcPacketSize);
    cmd.AddValue("mmTcInterval", "mMTC interval (s)", mmTcInterval);
    cmd.AddValue("numerology", "NR numerology", numerology);
    cmd.AddValue("centralFrequency", "Central frequency (Hz)", centralFrequency);
    cmd.AddValue("bandwidth", "Bandwidth (Hz)", bandwidth);
    cmd.AddValue("totalTxPower", "TX power (dBm)", totalTxPower);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("enableLogging", "Enable logging", enableLogging);
    cmd.AddValue("embbStaticWeight", "eMBB static weight", embbStaticWeight);
    cmd.AddValue("mmtcStaticWeight", "mMTC static weight", mmtcStaticWeight);
    cmd.AddValue("embbDynamicShare", "eMBB dynamic share", embbDynamicShare);
    cmd.AddValue("mmtcDynamicShare", "mMTC dynamic share", mmtcDynamicShare);
    cmd.AddValue("embbSlaThroughputMbps", "eMBB SLA throughput (Mbps)", embbSlaThroughputMbps);
    cmd.AddValue("embbSlaLatencyMs", "eMBB SLA latency (ms)", embbSlaLatencyMs);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(nEmbbUes + nMmTcUes != nUes,
                    "nEmbbUes + nMmTcUes must equal nUes");

    g_nEmbbUes = nEmbbUes;
    g_nMmTcUes = nMmTcUes;

    if (enableLogging)
    {
        LogComponentEnable("OranSlicingPrb", LOG_LEVEL_INFO);
        LogComponentEnable("NrMacSchedulerOfdmaSliceQos", LOG_LEVEL_DEBUG);
    }

    SystemPath::MakeDirectories(outputDir);
    RngSeedManager::SetSeed(runNumber);
    RngSeedManager::SetRun(runNumber);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue(999999999));

    NS_LOG_INFO("=== O-RAN SLICE-AWARE PRB SCHEDULING (1 BWP) ===");

    // EPC + Helper setup
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    // Use the slice-aware scheduler
    nrHelper->SetSchedulerTypeId(
        TypeId::LookupByName("ns3::NrMacSchedulerOfdmaSliceQos"));

    // Configure 1 shared BWP
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    bandConf.m_numBwp = 1; // Single shared BWP (not 2 like baseline)
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    if (enableLogging)
    {
        NS_LOG_INFO("BWP Config: 1 shared BWP, "
                    << allBwps[0].get()->m_channelBandwidth / 1e6 << " MHz @ "
                    << allBwps[0].get()->m_centralFrequency / 1e9 << " GHz");
    }

    // Antennas
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));

    // Nodes
    NodeContainer gnbNodes, ueEmbbNodes, ueMmTcNodes;
    gnbNodes.Create(1);
    ueEmbbNodes.Create(nEmbbUes);
    ueMmTcNodes.Create(nMmTcUes);

    // Mobility
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    Ptr<ConstantPositionMobilityModel> gnbPos =
        gnbNodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
    gnbPos->SetPosition(Vector(0.0, 0.0, 10.0));

    double angleStep = 2.0 * M_PI / nUes;
    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        mobility.Install(ueEmbbNodes.Get(i));
        Ptr<ConstantPositionMobilityModel> uePos =
            ueEmbbNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        uePos->SetPosition(Vector(ueDistance * std::cos(i * angleStep),
                                  ueDistance * std::sin(i * angleStep), 1.5));
    }
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        mobility.Install(ueMmTcNodes.Get(i));
        uint32_t gi = i + nEmbbUes;
        Ptr<ConstantPositionMobilityModel> uePos =
            ueMmTcNodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
        uePos->SetPosition(Vector(ueDistance * std::cos(gi * angleStep),
                                  ueDistance * std::sin(gi * angleStep), 1.5));
    }

    // Set scheduler attributes
    nrHelper->SetSchedulerAttribute("EmbbStaticWeight", DoubleValue(embbStaticWeight));
    nrHelper->SetSchedulerAttribute("MmtcStaticWeight", DoubleValue(mmtcStaticWeight));
    nrHelper->SetSchedulerAttribute("EmbbDynamicShare", DoubleValue(embbDynamicShare));
    nrHelper->SetSchedulerAttribute("MmtcDynamicShare", DoubleValue(mmtcDynamicShare));

    // Install devices
    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueEmbbDevs = nrHelper->InstallUeDevice(ueEmbbNodes, allBwps);
    NetDeviceContainer ueMmTcDevs = nrHelper->InstallUeDevice(ueMmTcNodes, allBwps);

    // Internet
    InternetStackHelper internet;
    internet.Install(ueEmbbNodes);
    internet.Install(ueMmTcNodes);
    internet.Install(gnbNodes);

    Ipv4InterfaceContainer ueEmbbIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueEmbbDevs);
    Ipv4InterfaceContainer ueMmTcIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueMmTcDevs);

    // PHY config
    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbPhy> gnbPhy = nrHelper->GetGnbPhy(gnbDevs.Get(i), 0);
        gnbPhy->SetAttribute("Numerology", UintegerValue(numerology));
        gnbPhy->SetAttribute("TxPower", DoubleValue(totalTxPower));
    }

    // Attach
    nrHelper->AttachToClosestGnb(ueEmbbDevs, gnbDevs);
    nrHelper->AttachToClosestGnb(ueMmTcDevs, gnbDevs);

    // Remote host / EPC
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

    for (uint32_t i = 0; i < ueEmbbNodes.GetN(); ++i)
    {
        Ptr<Ipv4StaticRouting> ueRouting =
            ipv4RoutingHelper.GetStaticRouting(ueEmbbNodes.Get(i)->GetObject<Ipv4>());
        ueRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 0);
    }
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        Ptr<Ipv4StaticRouting> ueRouting =
            ipv4RoutingHelper.GetStaticRouting(ueMmTcNodes.Get(i)->GetObject<Ipv4>());
        ueRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 0);
    }

    // Build global maps
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

    // Applications
    uint16_t dlPortEmbb = 1234;
    uint16_t ulPortMmtc = 5678;
    ApplicationContainer serverApps, clientApps;

    // eMBB downlink traffic
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

        // QCI 6 bearer -> maps to slice 0 in scheduler
        Ptr<NrEpcTft> embbTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter embbFilter;
        embbFilter.localPortStart = dlPortEmbb + i;
        embbFilter.localPortEnd = dlPortEmbb + i;
        embbTft->Add(embbFilter);
        NrEpsBearer embbBearer(NrEpsBearer::NGBR_VIDEO_TCP_OPERATOR);
        nrHelper->ActivateDedicatedEpsBearer(ueEmbbDevs.Get(i), embbBearer, embbTft);
    }

    // mMTC traffic (downlink, same as baseline workaround)
    for (uint32_t i = 0; i < ueMmTcNodes.GetN(); ++i)
    {
        uint16_t port = ulPortMmtc + i;
        Ipv4Address ueAddr = ueMmTcIpIfaces.GetAddress(i);

        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        serverApps.Add(sinkHelper.Install(ueMmTcNodes.Get(i)));

        UdpClientHelper dlClient(ueAddr, port);
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("PacketSize", UintegerValue(mmTcPacketSize));
        dlClient.SetAttribute("Interval", TimeValue(Seconds(mmTcInterval)));
        clientApps.Add(dlClient.Install(remoteHost));

        // QCI 80 bearer -> maps to slice 1 in scheduler
        Ptr<NrEpcTft> mmtcTft = Create<NrEpcTft>();
        NrEpcTft::PacketFilter mmtcFilter;
        mmtcFilter.localPortStart = port;
        mmtcFilter.localPortEnd = port;
        mmtcTft->Add(mmtcFilter);
        NrEpsBearer mmtcBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
        nrHelper->ActivateDedicatedEpsBearer(ueMmTcDevs.Get(i), mmtcBearer, mmtcTft);
    }

    serverApps.Start(Seconds(0.0));
    serverApps.Stop(Seconds(simTime));
    clientApps.Start(Seconds(0.1));
    clientApps.Stop(Seconds(simTime));

    // FlowMonitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("StartTime", TimeValue(Seconds(0.0)));

    NS_LOG_INFO("Starting simulation...");

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    Simulator::Destroy();

    // Output files
    std::string ueMetricsFile = outputDir + "/ue_metrics.csv";
    std::string sliceMetricsFile = outputDir + "/slice_metrics.csv";
    std::string timeseriesFile = outputDir + "/timeseries_metrics.csv";
    std::string summaryFile = outputDir + "/summary.json";

    // Process flows
    std::map<uint32_t, UeAgg> ueAggMap;
    SliceAgg slices[2];
    slices[0] = {0, "eMBB"};
    slices[1] = {1, "mMTC"};

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

    // UE metrics CSV
    std::ofstream ueOut(ueMetricsFile);
    ueOut << "ueId,sliceId,throughputMbps,avgDelayMs,packetLossRatio,"
          << "txBytes,rxBytes,txPackets,rxPackets\n";
    for (auto const& [ueId, ue] : ueAggMap)
    {
        double tp = ue.rxBytes * 8.0 / (simTime * 1e6);
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

    // Slice metrics CSV
    std::ofstream sliceOut(sliceMetricsFile);
    sliceOut << "sliceId,sliceName,throughputAggregatedMbps,throughputPerUeMbps,"
             << "avgLatencyMs,packetDeliveryRatio,packetLossRatio,"
             << "activeFlows,totalTxBytes,totalRxBytes,effectiveShare\n";

    double embbTp = 0, mmtcTp = 0;
    double embbLat = 0, mmtcLat = 0;
    double embbPdr = 0, mmtcPdr = 0;

    for (int s = 0; s < 2; ++s)
    {
        double tp = slices[s].rxBytes * 8.0 / (simTime * 1e6);
        uint32_t nUe = (s == 0) ? nEmbbUes : nMmTcUes;
        double tpUe = (nUe > 0) ? tp / nUe : 0.0;
        double lat = (slices[s].rxPackets > 0)
            ? (slices[s].delaySum / slices[s].rxPackets * 1000.0)
            : 0.0;
        double pdr = (slices[s].txPackets > 0)
            ? static_cast<double>(slices[s].rxPackets) / slices[s].txPackets
            : 0.0;

        double effShare = (s == 0)
            ? embbStaticWeight * 0.5 + embbDynamicShare * 0.5
            : mmtcStaticWeight * 0.5 + mmtcDynamicShare * 0.5;

        sliceOut << s << "," << slices[s].sliceName << ","
                 << tp << "," << tpUe << ","
                 << lat << "," << pdr << "," << (1.0 - pdr) << ","
                 << slices[s].activeFlows << ","
                 << slices[s].txBytes << "," << slices[s].rxBytes << ","
                 << effShare << "\n";

        if (s == 0)
        {
            embbTp = tp;
            embbLat = lat;
            embbPdr = pdr;
        }
        else
        {
            mmtcTp = tp;
            mmtcLat = lat;
            mmtcPdr = pdr;
        }
    }
    sliceOut.close();

    // Time series (cumulative final sample)
    std::ofstream tsOut(timeseriesFile);
    tsOut << "timestampSeconds,sliceId,sliceName,throughputMbps,"
          << "avgDelayMs,packetLossRatio,txBytes,rxBytes,"
          << "txPackets,rxPackets,activeUes\n";
    for (int s = 0; s < 2; ++s)
    {
        double tp = slices[s].rxBytes * 8.0 / (simTime * 1e6);
        double lat = (slices[s].rxPackets > 0)
            ? (slices[s].delaySum / slices[s].rxPackets * 1000.0)
            : 0.0;
        double plr = (slices[s].txPackets > 0)
            ? static_cast<double>(slices[s].txPackets - slices[s].rxPackets) / slices[s].txPackets
            : 0.0;

        tsOut << simTime << "," << s << "," << slices[s].sliceName << ","
              << tp << "," << lat << "," << plr << ","
              << slices[s].txBytes << "," << slices[s].rxBytes << ","
              << slices[s].txPackets << "," << slices[s].rxPackets << ","
              << ((s == 0) ? nEmbbUes : nMmTcUes) << "\n";
    }
    tsOut.close();

    // Summary JSON
    double embbTpUe = (nEmbbUes > 0) ? embbTp / nEmbbUes : 0.0;
    bool tpSla = (embbTpUe >= embbSlaThroughputMbps);
    bool latSla = (embbLat <= embbSlaLatencyMs);

    std::string bwpStr;
    {
        std::stringstream ss;
        ss << "1x" << allBwps[0].get()->m_channelBandwidth / 1e6 << "MHz";
        bwpStr = ss.str();
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
    sumOut << "    \"scheduler\": \"ns3::NrMacSchedulerOfdmaSliceQos\",\n";
    sumOut << "    \"bwpConfiguration\": \"" << bwpStr << "\",\n";
    sumOut << "    \"sliceImplementation\": \"slice_aware_rbg\",\n";
    sumOut << "    \"note\": \"1 BWP shared, slice-aware RBG partitioning at MAC\"\n";
    sumOut << "  },\n";
    sumOut << "  \"slicePolicy\": {\n";
    sumOut << "    \"staticPortion\": 0.5,\n";
    sumOut << "    \"dynamicPortion\": 0.5,\n";
    sumOut << "    \"embbStaticWeight\": " << embbStaticWeight << ",\n";
    sumOut << "    \"mmtcStaticWeight\": " << mmtcStaticWeight << ",\n";
    sumOut << "    \"embbDynamicShare\": " << embbDynamicShare << ",\n";
    sumOut << "    \"mmtcDynamicShare\": " << mmtcDynamicShare << ",\n";
    sumOut << "    \"embbEffectiveShare\": "
           << (embbStaticWeight * 0.5 + embbDynamicShare * 0.5) << ",\n";
    sumOut << "    \"mmtcEffectiveShare\": "
           << (mmtcStaticWeight * 0.5 + mmtcDynamicShare * 0.5) << "\n";
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
    sumOut << "  \"mMTC\": {\n";
    sumOut << "    \"throughputAggregatedMbps\": " << mmtcTp << ",\n";
    sumOut << "    \"avgLatencyMs\": " << mmtcLat << ",\n";
    sumOut << "    \"packetDeliveryRatio\": " << mmtcPdr << ",\n";
    sumOut << "    \"packetLossRatio\": " << (1.0 - mmtcPdr) << ",\n";
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
    sumOut << "  \"limitations\": [\n";
    sumOut << "    \"RBG-level granularity (not PRB-level) - native 5G-LENA constraint\",\n";
    sumOut << "    \"Slice-aware DL only; UL uses base QoS scheduler\",\n";
    sumOut << "    \"2 slices maximum in this implementation\",\n";
    sumOut << "    \"No DRL/xApp integration yet - parameters set at startup\",\n";
    sumOut << "    \"Metrics from FlowMonitor (IP layer), not MAC/PHY level\"\n";
    sumOut << "  ],\n";
    sumOut << "  \"technicalNotes\": {\n";
    sumOut << "    \"scheduler\": \"NrMacSchedulerOfdmaSliceQos\",\n";
    sumOut << "    \"inheritance\": \"NrMacSchedulerOfdmaRR -> NrMacSchedulerOfdmaQos -> SliceQos\",\n";
    sumOut << "    \"sliceIdentification\": \"QCI-based (eMBB:QCI=6, mMTC:QCI=80)\",\n";
    sumOut << "    \"resourcePartitioning\": \"50% static (weighted) + 50% dynamic\",\n";
    sumOut << "    \"workConserving\": true,\n";
    sumOut << "    \"drlReady\": \"SetSliceStaticWeight / SetSliceDynamicShare for xApp\"\n";
    sumOut << "  }\n";
    sumOut << "}\n";
    sumOut.close();

    // Console
    std::cout << "\n=================================================================\n";
    std::cout << "O-RAN SLICE-AWARE PRB SCHEDULING - SUMMARY\n";
    std::cout << "=================================================================\n";
    std::cout << "Time: " << simTime << "s | Run: " << runNumber << "\n";
    std::cout << "UEs: " << nUes << " (eMBB:" << nEmbbUes << " mMTC:" << nMmTcUes << ")\n";
    std::cout << "BWP: " << bwpStr << " shared | Scheduler: SliceQos\n";
    std::cout << "Shares: eMBB=" << (embbStaticWeight * 0.5 + embbDynamicShare * 0.5)
              << " mMTC=" << (mmtcStaticWeight * 0.5 + mmtcDynamicShare * 0.5) << "\n\n";
    std::cout << "eMBB: TP=" << embbTp << " Mbps (" << embbTpUe << " Mbps/UE)"
              << " Lat=" << embbLat << " ms PDR=" << (embbPdr * 100) << "%\n";
    std::cout << "  SLA TP>=" << embbSlaThroughputMbps << ": " << (tpSla ? "YES" : "NO")
              << " | SLA Lat<=" << embbSlaLatencyMs << ": " << (latSla ? "YES" : "NO") << "\n";
    std::cout << "mMTC: TP=" << mmtcTp << " Mbps"
              << " Lat=" << mmtcLat << " ms PDR=" << (mmtcPdr * 100) << "%\n\n";
    std::cout << "Files: " << outputDir << "/{ue_metrics,slice_metrics,timeseries_metrics,summary}.csv/.json\n";
    std::cout << "=================================================================\n";

    return 0;
}
