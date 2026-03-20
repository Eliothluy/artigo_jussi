// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#pragma once

#include "nr-mac-scheduler-ofdma-qos.h"

#include "ns3/traced-value.h"

#include <array>
#include <vector>

namespace ns3
{

/**
 * @ingroup scheduler
 * @brief OFDMA scheduler with slice-aware RBG partitioning (RSLAQ-inspired)
 *
 * Extends NrMacSchedulerOfdmaQos to provide resource partitioning at the
 * RBG level between network slices, rather than using physical BWP isolation.
 *
 * Architecture (RSLAQ-inspired):
 *   Total resources = static portion (50%) + dynamic portion (50%)
 *   Static portion: distributed by operator-configured weights
 *   Dynamic portion: distributed by configurable shares
 *   Work-conserving: unused resources from one slice redistributed
 *
 * Slice identification via QCI of the EPS bearer:
 *   QCI 6  (NGBR_VIDEO_TCP_OPERATOR) -> Slice 0 (eMBB)
 *   QCI 80 (NGBR_LOW_LAT_EMBB)       -> Slice 1 (mMTC)
 *
 * Limitations:
 *   - Granularity is RBG-level (native 5G-LENA), not PRB-level
 *   - Maximum 2 slices
 *   - Slice-aware DL and UL
 */
class NrMacSchedulerOfdmaSliceQos : public NrMacSchedulerOfdmaQos
{
  public:
    static TypeId GetTypeId();
    NrMacSchedulerOfdmaSliceQos();
    ~NrMacSchedulerOfdmaSliceQos() override;

    static constexpr uint8_t SLICE_EMBB = 0;
    static constexpr uint8_t SLICE_MMTC = 1;
    static constexpr uint8_t SLICE_COUNT = 2;

    struct SliceMetrics
    {
        uint32_t bytesToTransmit{0};
        uint32_t bufferStatus{0};
        double resourceShare{0.0};
        uint32_t txBytes{0};
        uint32_t droppedBytes{0};
        uint32_t activeUes{0};
        uint32_t allocatedRbgs{0};
    };

    SliceMetrics GetSliceMetrics(uint8_t slice) const;
    void SetSliceStaticWeight(uint8_t slice, double weight);
    void SetSliceDynamicShare(uint8_t slice, double share);
    double GetEffectiveShare(uint8_t slice) const;

  protected:
    std::shared_ptr<NrMacSchedulerUeInfo> CreateUeRepresentation(
        const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) const override;

    BeamSymbolMap AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const override;
    BeamSymbolMap AssignULRBG(uint32_t symAvail, const ActiveUeMap& activeUl) const override;

    std::function<bool(const UePtrAndBufferReq& lhs, const UePtrAndBufferReq& rhs)>
    GetUeCompareDlFn() const override;

    std::function<bool(const UePtrAndBufferReq& lhs, const UePtrAndBufferReq& rhs)>
    GetUeCompareUlFn() const override;

  private:
    uint8_t GetSliceIdFromUe(const UePtrAndBufferReq& ue) const;
    uint8_t GetMinQci(const UePtr& ue) const;

    std::array<std::vector<UePtrAndBufferReq>, SLICE_COUNT> ClassifyUesBySlice(
        const ActiveUeMap& activeUeMap) const;

    std::array<uint32_t, SLICE_COUNT> CalculateRbgQuota(
        uint32_t totalRbgs,
        const std::array<std::vector<UePtrAndBufferReq>, SLICE_COUNT>& sliceUes) const;

    void AllocateRbgsToSlice(uint8_t sliceId,
                             std::vector<UePtrAndBufferReq>& ueVector,
                             uint32_t rbgQuota,
                             uint32_t beamSym,
                             std::vector<bool>& availableRbgs,
                             FTResources& assignedResources) const;

    void AllocateUlRbgsToSlice(uint8_t sliceId,
                               std::vector<UePtrAndBufferReq>& ueVector,
                               uint32_t rbgQuota,
                               uint32_t beamSym,
                               std::vector<bool>& availableRbgs,
                               FTResources& assignedResources) const;

    double m_embbStaticWeight{0.5};
    double m_mmtcStaticWeight{0.5};
    double m_embbDynamicShare{0.5};
    double m_mmtcDynamicShare{0.5};
    double m_staticPortion{0.5};
    uint8_t m_embbQci{6};
    uint8_t m_mmtcQci{80};

    mutable std::array<SliceMetrics, SLICE_COUNT> m_sliceMetrics;
};

} // namespace ns3
