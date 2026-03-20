#pragma once

#include "ns3/object.h"
#include "ns3/oran-interface.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-bearer-stats-calculator.h"

#include <map>

namespace ns3
{

class E2InterfaceSlicing : public Object
{
  public:
    E2InterfaceSlicing();
    E2InterfaceSlicing(Ptr<NetDevice> netDev);
    ~E2InterfaceSlicing() override = default;
    static TypeId GetTypeId();
    void FunctionServiceSubscriptionCallback(E2AP_PDU_t* sub_req_pdu);
    void ControlMessageReceivedCallback(E2AP_PDU_t* sub_req_pdu);
    void BuildAndSendReportMessage(E2Termination::RicSubscriptionRequest_rval_s params);
    void ReportTxPDU(uint16_t rnti, uint8_t lcid, uint32_t packetSize);
    void SetE2PdcpStatsCalculator(Ptr<NrBearerStatsCalculator> calc);
    void SetE2RlcStatsCalculator(Ptr<NrBearerStatsCalculator> calc);

  private:
    Ptr<KpmIndicationHeader> BuildRicIndicationHeader(std::string plmId, std::string gnbId, uint16_t nrCellId);
    Ptr<KpmIndicationMessage> BuildRicIndicationMessageCuUp(std::string plmId);
    Ptr<KpmIndicationMessage> BuildRicIndicationMessageCuCp(std::string plmId);
    Ptr<KpmIndicationMessage> BuildRicIndicationMessageDu(std::string plmId, uint16_t nrCellId);
    std::string GetImsiString(uint64_t imsi);

    double m_e2Periodicity{0.1};
    Ptr<NetDevice> m_netDev;
    Ptr<NrGnbRrc> m_rrc;
    Ptr<E2Termination> m_e2term;
    Ptr<NrBearerStatsCalculator> m_e2PdcpStatsCalculator;
    Ptr<NrBearerStatsCalculator> m_e2RlcStatsCalculator;
    uint16_t m_cellId{0};
    std::map<uint32_t, uint32_t> m_txPDU;
    std::map<uint32_t, uint64_t> m_txPDUBytes;
    std::map<uint64_t, double> m_cellTxBytes;
    double m_cellRxBytes{0};
};

} // namespace ns3
