#include "E2-interface-slicing.h"

#include "ns3/kpm-indication.h"
#include "ns3/mmwave-indication-message-helper.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/config.h"
#include "ns3/nstime.h"
#include "ns3/pointer.h"
#include "ns3/double.h"

#include "encode_e2apv1.hpp"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("E2InterfaceSlicing");
NS_OBJECT_ENSURE_REGISTERED(E2InterfaceSlicing);

E2InterfaceSlicing::E2InterfaceSlicing()
{
    NS_FATAL_ERROR("E2InterfaceSlicing must be created with a net device");
}

E2InterfaceSlicing::E2InterfaceSlicing(Ptr<NetDevice> netDev)
{
    NS_LOG_FUNCTION(this);
    m_netDev = netDev;
    auto gnbDev = DynamicCast<NrGnbNetDevice>(netDev);
    if (gnbDev)
    {
        m_rrc = gnbDev->GetRrc();
    }
}

TypeId
E2InterfaceSlicing::GetTypeId()
{
    static TypeId tid = TypeId("ns3::E2InterfaceSlicing")
        .SetParent<Object>()
        .AddConstructor<E2InterfaceSlicing>()
        .AddAttribute("E2Term", "E2 termination instance",
            PointerValue(), MakePointerAccessor(&E2InterfaceSlicing::m_e2term),
            MakePointerChecker<E2Termination>())
        .AddAttribute("E2Periodicity", "E2 report periodicity (seconds)",
            DoubleValue(0.1), MakeDoubleAccessor(&E2InterfaceSlicing::m_e2Periodicity),
            MakeDoubleChecker<double>());
    return tid;
}

void
E2InterfaceSlicing::SetE2PdcpStatsCalculator(Ptr<NrBearerStatsCalculator> calc)
{
    m_e2PdcpStatsCalculator = calc;
}

void
E2InterfaceSlicing::SetE2RlcStatsCalculator(Ptr<NrBearerStatsCalculator> calc)
{
    m_e2RlcStatsCalculator = calc;
}

void
E2InterfaceSlicing::ReportTxPDU(uint16_t rnti, uint8_t lcid, uint32_t packetSize)
{
    m_txPDU[rnti] = (m_txPDU.find(rnti) == m_txPDU.end()) ? 1 : m_txPDU[rnti] + 1;
    m_txPDUBytes[rnti] = (m_txPDUBytes.find(rnti) == m_txPDUBytes.end()) ? packetSize : m_txPDUBytes[rnti] + packetSize;
}

void
E2InterfaceSlicing::FunctionServiceSubscriptionCallback(E2AP_PDU_t* sub_req_pdu)
{
    NS_LOG_FUNCTION(this);
    auto params = m_e2term->ProcessRicSubscriptionRequest(sub_req_pdu);
    NS_LOG_INFO("KPM Subscription received");
    BuildAndSendReportMessage(params);
}

void
E2InterfaceSlicing::BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params)
{
    NS_LOG_FUNCTION(this);
    auto e2Term = m_netDev->GetObject<E2Termination>();
    NS_ASSERT(e2Term != nullptr);

    std::string plmId = "111";
    auto gnbNode = DynamicCast<NrGnbNetDevice>(m_netDev);
    NS_ASSERT(gnbNode);
    m_cellId = gnbNode->GetCellId();
    std::string gnbId = std::to_string(m_cellId);

    // CU-UP
    auto header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
    auto cuUpMsg = BuildRicIndicationMessageCuUp(plmId);
    if (header && cuUpMsg)
    {
        auto pdu = new E2AP_PDU;
        encoding::generate_e2apv1_indication_request_parameterized(
            pdu, params.requestorId, params.instanceId, params.ranFuncionId,
            params.actionId, 1, (uint8_t*)header->m_buffer, header->m_size,
            (uint8_t*)cuUpMsg->m_buffer, cuUpMsg->m_size);
        e2Term->SendE2Message(pdu);
        delete pdu;
    }

    // DU
    header = BuildRicIndicationHeader(plmId, gnbId, m_cellId);
    auto duMsg = BuildRicIndicationMessageDu(plmId, m_cellId);
    if (header && duMsg)
    {
        auto pdu = new E2AP_PDU;
        encoding::generate_e2apv1_indication_request_parameterized(
            pdu, params.requestorId, params.instanceId, params.ranFuncionId,
            params.actionId, 1, (uint8_t*)header->m_buffer, header->m_size,
            (uint8_t*)duMsg->m_buffer, duMsg->m_size);
        m_e2term->SendE2Message(pdu);
        delete pdu;
    }

    Simulator::ScheduleWithContext(1, Seconds(m_e2Periodicity),
        &E2InterfaceSlicing::BuildAndSendReportMessage, this, params);
}

Ptr<KpmIndicationHeader>
E2InterfaceSlicing::BuildRicIndicationHeader(std::string plmId, std::string gnbId, uint16_t nrCellId)
{
    KpmIndicationHeader::KpmRicIndicationHeaderValues headerValues;
    headerValues.m_plmId = plmId;
    headerValues.m_gnbId = gnbId;
    headerValues.m_nrCellId = nrCellId;
    headerValues.m_timestamp = Simulator::Now().GetMilliSeconds();
    return Create<KpmIndicationHeader>(KpmIndicationHeader::GlobalE2nodeType::gNB, headerValues);
}

Ptr<KpmIndicationMessage>
E2InterfaceSlicing::BuildRicIndicationMessageCuUp(std::string plmId)
{
    auto helper = CreateObject<MmWaveIndicationMessageHelper>(
        IndicationMessageHelper::IndicationMessageType::CuUp, false, false);

    helper->AddCuUpUePmItem("UE-0", 100, 10);
    helper->FillCuUpValues(plmId);
    return helper->CreateIndicationMessage();
}

Ptr<KpmIndicationMessage>
E2InterfaceSlicing::BuildRicIndicationMessageCuCp(std::string plmId)
{
    auto helper = CreateObject<MmWaveIndicationMessageHelper>(
        IndicationMessageHelper::IndicationMessageType::CuCp, false, false);

    helper->FillCuCpValues(1);
    return helper->CreateIndicationMessage();
}

Ptr<KpmIndicationMessage>
E2InterfaceSlicing::BuildRicIndicationMessageDu(std::string plmId, uint16_t nrCellId)
{
    auto helper = CreateObject<MmWaveIndicationMessageHelper>(
        IndicationMessageHelper::IndicationMessageType::Du, false, false);

    Ptr<CellResourceReport> cellResRep = Create<CellResourceReport>();
    cellResRep->m_plmId = plmId;
    cellResRep->m_nrCellId = nrCellId;
    cellResRep->dlAvailablePrbs = 3333;
    cellResRep->ulAvailablePrbs = 3333;

    Ptr<ServedPlmnPerCell> servedPlmn = Create<ServedPlmnPerCell>();
    servedPlmn->m_plmId = plmId;
    servedPlmn->m_nrCellId = nrCellId;

    Ptr<EpcDuPmContainer> epcDu = Create<EpcDuPmContainer>();
    epcDu->m_qci = 1;
    epcDu->m_dlPrbUsage = 50;
    epcDu->m_ulPrbUsage = 50;

    servedPlmn->m_perQciReportItems.insert(epcDu);
    cellResRep->m_servedPlmnPerCellItems.insert(servedPlmn);
    helper->AddDuCellResRepPmItem(cellResRep);
    helper->FillDuValues(plmId + std::to_string(nrCellId));

    return helper->CreateIndicationMessage();
}

void
E2InterfaceSlicing::ControlMessageReceivedCallback(E2AP_PDU_t* sub_req_pdu)
{
    NS_LOG_INFO("Received RIC Control Message");
    Ptr<RicControlMessage> controlMessage = Create<RicControlMessage>(sub_req_pdu);
    NS_LOG_INFO("Request type: " << controlMessage->m_requestType);
}

} // namespace ns3
