// Copyright (c) 2024 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#include "nr-mac-scheduler-ofdma-slice-qos.h"

#include "nr-mac-scheduler-ue-info-qos.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <numeric>
#include <set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrMacSchedulerOfdmaSliceQos");
NS_OBJECT_ENSURE_REGISTERED(NrMacSchedulerOfdmaSliceQos);

TypeId
NrMacSchedulerOfdmaSliceQos::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrMacSchedulerOfdmaSliceQos")
            .SetParent<NrMacSchedulerOfdmaQos>()
            .AddConstructor<NrMacSchedulerOfdmaSliceQos>()
            .AddAttribute("EmbbStaticWeight",
                          "Static resource weight for eMBB slice (0-1)",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaSliceQos::m_embbStaticWeight),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("MmtcStaticWeight",
                          "Static resource weight for mMTC slice (0-1)",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaSliceQos::m_mmtcStaticWeight),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("EmbbDynamicShare",
                          "Dynamic resource share for eMBB slice (0-1)",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaSliceQos::m_embbDynamicShare),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("MmtcDynamicShare",
                          "Dynamic resource share for mMTC slice (0-1)",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaSliceQos::m_mmtcDynamicShare),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("StaticPortion",
                          "Portion of resources allocated statically (0-1)",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&NrMacSchedulerOfdmaSliceQos::m_staticPortion),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("EmbbQci",
                          "QCI value identifying eMBB slice",
                          UintegerValue(6),
                          MakeUintegerAccessor(&NrMacSchedulerOfdmaSliceQos::m_embbQci),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("MmtcQci",
                          "QCI value identifying mMTC slice",
                          UintegerValue(80),
                          MakeUintegerAccessor(&NrMacSchedulerOfdmaSliceQos::m_mmtcQci),
                          MakeUintegerChecker<uint8_t>());
    return tid;
}

NrMacSchedulerOfdmaSliceQos::NrMacSchedulerOfdmaSliceQos()
    : NrMacSchedulerOfdmaQos()
{
    NS_LOG_FUNCTION(this);
}

NrMacSchedulerOfdmaSliceQos::~NrMacSchedulerOfdmaSliceQos()
{
    NS_LOG_FUNCTION(this);
}

NrMacSchedulerOfdmaSliceQos::SliceMetrics
NrMacSchedulerOfdmaSliceQos::GetSliceMetrics(uint8_t slice) const
{
    if (slice >= SLICE_COUNT)
    {
        return SliceMetrics{};
    }
    return m_sliceMetrics[slice];
}

void
NrMacSchedulerOfdmaSliceQos::SetSliceStaticWeight(uint8_t slice, double weight)
{
    NS_LOG_FUNCTION(this << slice << weight);
    if (slice == SLICE_EMBB)
    {
        m_embbStaticWeight = weight;
        m_mmtcStaticWeight = 1.0 - weight;
    }
    else if (slice == SLICE_MMTC)
    {
        m_mmtcStaticWeight = weight;
        m_embbStaticWeight = 1.0 - weight;
    }
}

void
NrMacSchedulerOfdmaSliceQos::SetSliceDynamicShare(uint8_t slice, double share)
{
    NS_LOG_FUNCTION(this << slice << share);
    if (slice == SLICE_EMBB)
    {
        m_embbDynamicShare = share;
        m_mmtcDynamicShare = 1.0 - share;
    }
    else if (slice == SLICE_MMTC)
    {
        m_mmtcDynamicShare = share;
        m_embbDynamicShare = 1.0 - share;
    }
}

double
NrMacSchedulerOfdmaSliceQos::GetEffectiveShare(uint8_t slice) const
{
    double staticPart = m_staticPortion;
    double dynamicPart = 1.0 - m_staticPortion;

    if (slice == SLICE_EMBB)
    {
        return m_embbStaticWeight * staticPart + m_embbDynamicShare * dynamicPart;
    }
    else if (slice == SLICE_MMTC)
    {
        return m_mmtcStaticWeight * staticPart + m_mmtcDynamicShare * dynamicPart;
    }
    return 0.0;
}

std::shared_ptr<NrMacSchedulerUeInfo>
NrMacSchedulerOfdmaSliceQos::CreateUeRepresentation(
    const NrMacCschedSapProvider::CschedUeConfigReqParameters& params) const
{
    NS_LOG_FUNCTION(this);
    return std::make_shared<NrMacSchedulerUeInfoQos>(
        GetFairnessIndex(),
        params.m_rnti,
        params.m_beamId,
        std::bind(&NrMacSchedulerOfdmaSliceQos::GetNumRbPerRbg, this));
}

uint8_t
NrMacSchedulerOfdmaSliceQos::GetMinQci(const UePtr& ue) const
{
    uint8_t minQci = 255;

    // Check DL LCGs for QCI
    for (const auto& [lcgId, lcg] : ue->m_dlLCG)
    {
        for (const auto lcId : lcg->GetActiveLCIds())
        {
            const auto& lc = lcg->GetLC(lcId);
            if (lc->m_qci < minQci)
            {
                minQci = lc->m_qci;
            }
        }
    }

    // Also check UL LCGs (essential for UL-only bearers)
    for (const auto& [lcgId, lcg] : ue->m_ulLCG)
    {
        for (const auto lcId : lcg->GetActiveLCIds())
        {
            const auto& lc = lcg->GetLC(lcId);
            if (lc->m_qci < minQci)
            {
                minQci = lc->m_qci;
            }
        }
    }

    return minQci;
}

uint8_t
NrMacSchedulerOfdmaSliceQos::GetSliceIdFromUe(const UePtrAndBufferReq& ue) const
{
    uint8_t qci = GetMinQci(ue.first);

    if (qci == m_embbQci || qci == 6)
    {
        return SLICE_EMBB;
    }
    else if (qci == m_mmtcQci || qci == 80)
    {
        return SLICE_MMTC;
    }

    return (qci < 80) ? SLICE_EMBB : SLICE_MMTC;
}

std::array<std::vector<NrMacSchedulerOfdmaSliceQos::UePtrAndBufferReq>,
           NrMacSchedulerOfdmaSliceQos::SLICE_COUNT>
NrMacSchedulerOfdmaSliceQos::ClassifyUesBySlice(const ActiveUeMap& activeUeMap) const
{
    std::array<std::vector<UePtrAndBufferReq>, SLICE_COUNT> result;

    for (const auto& [beamId, ueVector] : activeUeMap)
    {
        for (const auto& ue : ueVector)
        {
            uint8_t sliceId = GetSliceIdFromUe(ue);
            result[sliceId].push_back(ue);
        }
    }
    return result;
}

std::array<uint32_t, NrMacSchedulerOfdmaSliceQos::SLICE_COUNT>
NrMacSchedulerOfdmaSliceQos::CalculateRbgQuota(
    uint32_t totalRbgs,
    const std::array<std::vector<UePtrAndBufferReq>, SLICE_COUNT>& sliceUes) const
{
    std::array<uint32_t, SLICE_COUNT> quota{0, 0};

    std::array<bool, SLICE_COUNT> hasUes{false, false};
    for (uint8_t s = 0; s < SLICE_COUNT; ++s)
    {
        hasUes[s] = !sliceUes[s].empty();
    }

    uint32_t staticRbgs = static_cast<uint32_t>(totalRbgs * m_staticPortion);
    uint32_t dynamicRbgs = totalRbgs - staticRbgs;

    // Static allocation (weighted)
    double totalStaticWeight = m_embbStaticWeight + m_mmtcStaticWeight;
    if (totalStaticWeight > 0)
    {
        quota[SLICE_EMBB] =
            static_cast<uint32_t>(staticRbgs * m_embbStaticWeight / totalStaticWeight);
        quota[SLICE_MMTC] = staticRbgs - quota[SLICE_EMBB];
    }

    // Dynamic allocation
    if (hasUes[SLICE_EMBB] && !hasUes[SLICE_MMTC])
    {
        quota[SLICE_EMBB] += dynamicRbgs;
    }
    else if (!hasUes[SLICE_EMBB] && hasUes[SLICE_MMTC])
    {
        quota[SLICE_MMTC] += dynamicRbgs;
    }
    else if (hasUes[SLICE_EMBB] && hasUes[SLICE_MMTC])
    {
        double totalDynShare = m_embbDynamicShare + m_mmtcDynamicShare;
        if (totalDynShare > 0)
        {
            uint32_t embbDyn =
                static_cast<uint32_t>(dynamicRbgs * m_embbDynamicShare / totalDynShare);
            uint32_t mmtcDyn = dynamicRbgs - embbDyn;
            quota[SLICE_EMBB] += embbDyn;
            quota[SLICE_MMTC] += mmtcDyn;
        }
        else
        {
            quota[SLICE_EMBB] += dynamicRbgs / 2;
            quota[SLICE_MMTC] += dynamicRbgs - dynamicRbgs / 2;
        }
    }

    for (uint8_t s = 0; s < SLICE_COUNT; ++s)
    {
        if (quota[s] > totalRbgs)
        {
            quota[s] = totalRbgs;
        }
    }

    NS_LOG_DEBUG("RBG quota: eMBB=" << quota[SLICE_EMBB] << " mMTC=" << quota[SLICE_MMTC]
                                     << " total=" << totalRbgs);

    return quota;
}

void
NrMacSchedulerOfdmaSliceQos::AllocateRbgsToSlice(
    uint8_t sliceId,
    std::vector<UePtrAndBufferReq>& ueVector,
    uint32_t rbgQuota,
    uint32_t beamSym,
    std::vector<bool>& availableRbgs,
    FTResources& assignedResources) const
{
    if (ueVector.empty() || rbgQuota == 0)
    {
        return;
    }

    SortUeVector(&ueVector,
                 std::bind(&NrMacSchedulerOfdmaSliceQos::GetUeCompareDlFn, this));

    std::set<uint32_t> remainingRbgSet;
    for (size_t i = 0; i < availableRbgs.size(); ++i)
    {
        if (availableRbgs[i])
        {
            remainingRbgSet.emplace(i);
        }
    }

    uint32_t allocatedCount = 0;

    while (allocatedCount < rbgQuota && !remainingRbgSet.empty())
    {
        SortUeVector(&ueVector,
                     std::bind(&NrMacSchedulerOfdmaSliceQos::GetUeCompareDlFn, this));

        auto schedInfoIt = ueVector.begin();

        while (schedInfoIt != ueVector.end())
        {
            uint32_t bufQueueSize = schedInfoIt->second;
            if (schedInfoIt->first->m_dlTbSize >= std::max(bufQueueSize, 10U))
            {
                std::advance(schedInfoIt, 1);
            }
            else
            {
                break;
            }
        }

        if (schedInfoIt == ueVector.end())
        {
            break;
        }

        auto rbgIt = remainingRbgSet.begin();
        if (rbgIt == remainingRbgSet.end())
        {
            break;
        }

        uint32_t currentRbg = *rbgIt;

        // Allocate RBG to UE
        auto& assignedRbgs = schedInfoIt->first->m_dlRBG;
        auto existingRbgs = assignedRbgs.size();
        assignedRbgs.resize(assignedRbgs.size() + beamSym);
        std::fill(assignedRbgs.begin() + existingRbgs, assignedRbgs.end(), currentRbg);
        assignedResources.m_rbg++;

        auto& assignedSymbols = schedInfoIt->first->m_dlSym;
        auto existingSymbols = assignedSymbols.size();
        assignedSymbols.resize(assignedSymbols.size() + beamSym);
        std::iota(assignedSymbols.begin() + existingSymbols, assignedSymbols.end(), 0);
        assignedResources.m_sym = beamSym;

        availableRbgs[currentRbg] = false;
        remainingRbgSet.erase(rbgIt);
        allocatedCount++;

        // Update UE metrics
        auto uePtr =
            std::dynamic_pointer_cast<NrMacSchedulerUeInfoQos>(schedInfoIt->first);
        if (uePtr)
        {
            uePtr->UpdateDlQosMetric(assignedResources, GetTimeWindow());
        }

        for (auto& ue : ueVector)
        {
            if (ue.first->m_rnti != schedInfoIt->first->m_rnti)
            {
                auto ueQos = std::dynamic_pointer_cast<NrMacSchedulerUeInfoQos>(ue.first);
                if (ueQos)
                {
                    ueQos->UpdateDlQosMetric(assignedResources, GetTimeWindow());
                }
            }
        }
    }

    m_sliceMetrics[sliceId].allocatedRbgs = allocatedCount;
    m_sliceMetrics[sliceId].activeUes = ueVector.size();
    m_sliceMetrics[sliceId].resourceShare = GetEffectiveShare(sliceId);
}

void
NrMacSchedulerOfdmaSliceQos::AllocateUlRbgsToSlice(
    uint8_t sliceId,
    std::vector<UePtrAndBufferReq>& ueVector,
    uint32_t rbgQuota,
    uint32_t beamSym,
    std::vector<bool>& availableRbgs,
    FTResources& assignedResources) const
{
    if (ueVector.empty() || rbgQuota == 0)
    {
        return;
    }

    SortUeVector(&ueVector,
                 std::bind(&NrMacSchedulerOfdmaSliceQos::GetUeCompareUlFn, this));

    std::set<uint32_t> remainingRbgSet;
    for (size_t i = 0; i < availableRbgs.size(); ++i)
    {
        if (availableRbgs[i])
        {
            remainingRbgSet.emplace(i);
        }
    }

    uint32_t allocatedCount = 0;

    while (allocatedCount < rbgQuota && !remainingRbgSet.empty())
    {
        SortUeVector(&ueVector,
                     std::bind(&NrMacSchedulerOfdmaSliceQos::GetUeCompareUlFn, this));

        auto schedInfoIt = ueVector.begin();

        while (schedInfoIt != ueVector.end())
        {
            uint32_t bufQueueSize = schedInfoIt->second;
            if (schedInfoIt->first->m_ulTbSize >= std::max(bufQueueSize, 12U))
            {
                std::advance(schedInfoIt, 1);
            }
            else
            {
                break;
            }
        }

        if (schedInfoIt == ueVector.end())
        {
            break;
        }

        auto rbgIt = remainingRbgSet.begin();
        if (rbgIt == remainingRbgSet.end())
        {
            break;
        }

        uint32_t currentRbg = *rbgIt;

        // Allocate RBG to UE (UL fields)
        auto& assignedRbgs = schedInfoIt->first->m_ulRBG;
        auto existingRbgs = assignedRbgs.size();
        assignedRbgs.resize(assignedRbgs.size() + beamSym);
        std::fill(assignedRbgs.begin() + existingRbgs, assignedRbgs.end(), currentRbg);
        assignedResources.m_rbg++;

        auto& assignedSymbols = schedInfoIt->first->m_ulSym;
        auto existingSymbols = assignedSymbols.size();
        assignedSymbols.resize(assignedSymbols.size() + beamSym);
        std::iota(assignedSymbols.begin() + existingSymbols, assignedSymbols.end(), 0);
        assignedResources.m_sym = beamSym;

        availableRbgs[currentRbg] = false;
        remainingRbgSet.erase(rbgIt);
        allocatedCount++;

        // Update UL metrics for the scheduled UE
        auto uePtr =
            std::dynamic_pointer_cast<NrMacSchedulerUeInfoQos>(schedInfoIt->first);
        if (uePtr)
        {
            uePtr->UpdateUlQosMetric(assignedResources, GetTimeWindow());
        }

        // Update metrics for unscheduled UEs
        for (auto& ue : ueVector)
        {
            if (ue.first->m_rnti != schedInfoIt->first->m_rnti)
            {
                auto ueQos = std::dynamic_pointer_cast<NrMacSchedulerUeInfoQos>(ue.first);
                if (ueQos)
                {
                    ueQos->UpdateUlQosMetric(assignedResources, GetTimeWindow());
                }
            }
        }
    }

    m_sliceMetrics[sliceId].allocatedRbgs += allocatedCount;
    m_sliceMetrics[sliceId].activeUes = ueVector.size();
    m_sliceMetrics[sliceId].resourceShare = GetEffectiveShare(sliceId);
}

NrMacSchedulerNs3::BeamSymbolMap
NrMacSchedulerOfdmaSliceQos::AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const
{
    NS_LOG_FUNCTION(this);

    for (uint8_t s = 0; s < SLICE_COUNT; ++s)
    {
        m_sliceMetrics[s] = SliceMetrics{};
    }

    GetFirst GetBeamId;
    GetSecond GetUeVector;
    BeamSymbolMap symPerBeam = GetSymPerBeam(symAvail, activeDl);

    for (const auto& el : activeDl)
    {
        uint32_t beamSym = symPerBeam.at(GetBeamId(el));
        FTResources assignedResources(0, 0);
        std::vector<bool> availableRbgs = GetDlBitmask();

        uint32_t totalRbgs = std::count(availableRbgs.begin(), availableRbgs.end(), true);

        // Classify UEs by slice
        ActiveUeMap singleBeamMap;
        singleBeamMap[GetBeamId(el)] = GetUeVector(el);
        auto sliceUes = ClassifyUesBySlice(singleBeamMap);

        // Calculate RBG quota per slice
        auto rbgQuota = CalculateRbgQuota(totalRbgs, sliceUes);

        // Allocate RBGs to each slice
        for (uint8_t s = 0; s < SLICE_COUNT; ++s)
        {
            if (!sliceUes[s].empty() && rbgQuota[s] > 0)
            {
                NS_LOG_DEBUG("Allocating " << rbgQuota[s] << " RBGs to slice " << s
                                           << " with " << sliceUes[s].size() << " UEs");

                for (auto& ue : sliceUes[s])
                {
                    BeforeDlSched(ue, FTResources(beamSym, beamSym));
                }

                AllocateRbgsToSlice(
                    s, sliceUes[s], rbgQuota[s], beamSym, availableRbgs, assignedResources);
            }
        }

        // Work-conserving: allocate remaining RBGs to any UE with data
        std::vector<UePtrAndBufferReq> allUes;
        for (uint8_t s = 0; s < SLICE_COUNT; ++s)
        {
            allUes.insert(allUes.end(), sliceUes[s].begin(), sliceUes[s].end());
        }

        if (!allUes.empty())
        {
            std::set<uint32_t> remainingRbgSet;
            for (size_t i = 0; i < availableRbgs.size(); ++i)
            {
                if (availableRbgs[i])
                {
                    remainingRbgSet.emplace(i);
                }
            }

            while (!remainingRbgSet.empty())
            {
                SortUeVector(&allUes,
                             std::bind(&NrMacSchedulerOfdmaSliceQos::GetUeCompareDlFn, this));

                auto schedInfoIt = allUes.begin();
                while (schedInfoIt != allUes.end())
                {
                    uint32_t bufQueueSize = schedInfoIt->second;
                    if (schedInfoIt->first->m_dlTbSize >= std::max(bufQueueSize, 10U))
                    {
                        std::advance(schedInfoIt, 1);
                    }
                    else
                    {
                        break;
                    }
                }

                if (schedInfoIt == allUes.end())
                {
                    break;
                }

                auto rbgIt = remainingRbgSet.begin();
                uint32_t currentRbg = *rbgIt;

                auto& assignedRbgs = schedInfoIt->first->m_dlRBG;
                auto existingRbgs = assignedRbgs.size();
                assignedRbgs.resize(assignedRbgs.size() + beamSym);
                std::fill(assignedRbgs.begin() + existingRbgs, assignedRbgs.end(), currentRbg);
                assignedResources.m_rbg++;

                auto& assignedSymbols = schedInfoIt->first->m_dlSym;
                auto existingSymbols = assignedSymbols.size();
                assignedSymbols.resize(assignedSymbols.size() + beamSym);
                std::iota(assignedSymbols.begin() + existingSymbols, assignedSymbols.end(), 0);
                assignedResources.m_sym = beamSym;

                availableRbgs[currentRbg] = false;
                remainingRbgSet.erase(rbgIt);

                auto uePtr =
                    std::dynamic_pointer_cast<NrMacSchedulerUeInfoQos>(schedInfoIt->first);
                if (uePtr)
                {
                    uePtr->UpdateDlQosMetric(assignedResources, GetTimeWindow());
                }
            }
        }
    }

    return symPerBeam;
}

NrMacSchedulerNs3::BeamSymbolMap
NrMacSchedulerOfdmaSliceQos::AssignULRBG(uint32_t symAvail, const ActiveUeMap& activeUl) const
{
    NS_LOG_FUNCTION(this);

    for (uint8_t s = 0; s < SLICE_COUNT; ++s)
    {
        m_sliceMetrics[s] = SliceMetrics{};
    }

    GetFirst GetBeamId;
    GetSecond GetUeVector;
    BeamSymbolMap symPerBeam = GetSymPerBeam(symAvail, activeUl);

    for (const auto& el : activeUl)
    {
        uint32_t beamSym = symPerBeam.at(GetBeamId(el));
        FTResources assignedResources(0, 0);
        std::vector<bool> availableRbgs = GetUlBitmask();

        uint32_t totalRbgs = std::count(availableRbgs.begin(), availableRbgs.end(), true);

        // Classify UEs by slice
        ActiveUeMap singleBeamMap;
        singleBeamMap[GetBeamId(el)] = GetUeVector(el);
        auto sliceUes = ClassifyUesBySlice(singleBeamMap);

        // Calculate RBG quota per slice (same RSLAQ policy as DL)
        auto rbgQuota = CalculateRbgQuota(totalRbgs, sliceUes);

        // Allocate RBGs to each slice
        for (uint8_t s = 0; s < SLICE_COUNT; ++s)
        {
            if (!sliceUes[s].empty() && rbgQuota[s] > 0)
            {
                NS_LOG_DEBUG("UL: Allocating " << rbgQuota[s] << " RBGs to slice " << s
                                               << " with " << sliceUes[s].size() << " UEs");

                for (auto& ue : sliceUes[s])
                {
                    BeforeUlSched(ue, FTResources(beamSym, beamSym));
                }

                AllocateUlRbgsToSlice(
                    s, sliceUes[s], rbgQuota[s], beamSym, availableRbgs, assignedResources);
            }
        }

        // Work-conserving: allocate remaining RBGs to any UE with data
        std::vector<UePtrAndBufferReq> allUes;
        for (uint8_t s = 0; s < SLICE_COUNT; ++s)
        {
            allUes.insert(allUes.end(), sliceUes[s].begin(), sliceUes[s].end());
        }

        if (!allUes.empty())
        {
            std::set<uint32_t> remainingRbgSet;
            for (size_t i = 0; i < availableRbgs.size(); ++i)
            {
                if (availableRbgs[i])
                {
                    remainingRbgSet.emplace(i);
                }
            }

            while (!remainingRbgSet.empty())
            {
                SortUeVector(&allUes,
                             std::bind(&NrMacSchedulerOfdmaSliceQos::GetUeCompareUlFn, this));

                auto schedInfoIt = allUes.begin();
                while (schedInfoIt != allUes.end())
                {
                    uint32_t bufQueueSize = schedInfoIt->second;
                    if (schedInfoIt->first->m_ulTbSize >= std::max(bufQueueSize, 12U))
                    {
                        std::advance(schedInfoIt, 1);
                    }
                    else
                    {
                        break;
                    }
                }

                if (schedInfoIt == allUes.end())
                {
                    break;
                }

                auto rbgIt = remainingRbgSet.begin();
                uint32_t currentRbg = *rbgIt;

                auto& assignedRbgs = schedInfoIt->first->m_ulRBG;
                auto existingRbgs = assignedRbgs.size();
                assignedRbgs.resize(assignedRbgs.size() + beamSym);
                std::fill(assignedRbgs.begin() + existingRbgs, assignedRbgs.end(), currentRbg);
                assignedResources.m_rbg++;

                auto& assignedSymbols = schedInfoIt->first->m_ulSym;
                auto existingSymbols = assignedSymbols.size();
                assignedSymbols.resize(assignedSymbols.size() + beamSym);
                std::iota(assignedSymbols.begin() + existingSymbols, assignedSymbols.end(), 0);
                assignedResources.m_sym = beamSym;

                availableRbgs[currentRbg] = false;
                remainingRbgSet.erase(rbgIt);

                auto uePtr =
                    std::dynamic_pointer_cast<NrMacSchedulerUeInfoQos>(schedInfoIt->first);
                if (uePtr)
                {
                    uePtr->UpdateUlQosMetric(assignedResources, GetTimeWindow());
                }
            }
        }
    }

    return symPerBeam;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaSliceQos::GetUeCompareDlFn() const
{
    return NrMacSchedulerUeInfoQos::CompareUeWeightsDl;
}

std::function<bool(const NrMacSchedulerNs3::UePtrAndBufferReq& lhs,
                   const NrMacSchedulerNs3::UePtrAndBufferReq& rhs)>
NrMacSchedulerOfdmaSliceQos::GetUeCompareUlFn() const
{
    return NrMacSchedulerUeInfoQos::CompareUeWeightsUl;
}

} // namespace ns3
