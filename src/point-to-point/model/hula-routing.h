/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2023 NUS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Chahwan Song <songch@comp.nus.edu.sg>
 */


#pragma once

#include <arpa/inet.h>

#include <map>
#include <queue>
#include <unordered_map>
#include <vector>

#include "ns3/address.h"
#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/net-device.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/settings.h"
#include "ns3/simulator.h"
#include "ns3/tag.h"

namespace ns3 {
/**
 * @brief Conga object is created for each ToR Switch
 */
class HulaRouting : public Object {
    friend class SwitchMmu;
    friend class SwitchNode;

   public:
    HulaRouting();
    /* static */
    static TypeId GetTypeId(void);
    static uint64_t GetQpKey(uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg);              // same as in rdma_hw.cc
    static uint32_t nFlowletTimeout;
    static std::vector<HulaRouting*> hulaModules;

    /* main function */
    void RouteInput(Ptr<Packet> p, CustomHeader ch);
    void processProbe(uint32_t inDev, Ptr<Packet> p, CustomHeader ch);
    void updateLink(uint32_t dev, uint32_t packetSize);
    void active(int time);

    /* SET functions */
    void SetSwitchInfo(bool isToR, uint32_t switch_id);
    void SetLinkCapacity(uint32_t outPort, uint64_t bitRate);
    void SetConstants(Time keepAliveThresh,
                            Time probeTransmitInterval,
                            Time flowletTimeout,
                            Time probeGenerationInterval,
                            Time tau);

    // topological info (should be initialized in the beginning)
    /*-----CALLBACK------*/
    void DoSwitchSend(Ptr<Packet> p, CustomHeader& ch, uint32_t outDev,
                      uint32_t qIndex);  // TxToR and Agg/CoreSw
    void DoSwitchSendToDev(Ptr<Packet> p, CustomHeader& ch);  // only at RxToR
    void DoSwitchSendHulaProbe(uint32_t dev, uint32_t torID, uint8_t minUtil);
    typedef Callback<void, Ptr<Packet>, CustomHeader&, uint32_t, uint32_t> SwitchSendCallback;
    typedef Callback<void, Ptr<Packet>, CustomHeader&> SwitchSendToDevCallback;
    typedef Callback<void, uint32_t, uint32_t, uint8_t> SwitchSendHulaProbeCallback;
    void SetSwitchSendCallback(SwitchSendCallback switchSendCallback);  // set callback
    void SetSwitchSendToDevCallback(
        SwitchSendToDevCallback switchSendToDevCallback);  // set callback
    void SetSwitchSendHulaProbeCallback(SwitchSendHulaProbeCallback switchSendHulaProbeCallback);
    /*-----------*/
    
    std::unordered_set<uint32_t> upLayerDevs;          
    std::unordered_set<uint32_t> downLayerDevs;

    void Print();

   private:
    // callback
    SwitchSendCallback m_switchSendCallback;  // bound to SwitchNode::SwitchSend (for Request/UDP)
    SwitchSendToDevCallback m_switchSendToDevCallback;  // bound to SwitchNode::SendToDevContinue (for Probe, Reply)
    SwitchSendHulaProbeCallback m_switchSendHulaProbeCallback;

    struct NextHopItem {
        uint32_t    nextHopDev;       //下一跳
        uint8_t     pathUtil;          //链路利用率
        Time        lastUpdateTime;   //上次更新时间
        Time        lastProbeSendTime;
        NextHopItem() :
            nextHopDev(0), 
            pathUtil(255), 
            lastUpdateTime(Seconds(0)),
            lastProbeSendTime(Seconds(-1)) {}
    };

    struct LinkInfo {
        uint64_t maxBitRate;
        uint64_t curUtil;
        Time lastUpdateTime;
        LinkInfo() : maxBitRate(0), curUtil(0), lastUpdateTime(Seconds(0)) {}
    };

    struct FlowletInfo {
        Time activeTime;       // to check creating a new flowlet
        Time createTime;    // start time of new flowlet
        uint32_t nextHopDev;    // current pathId
        uint32_t nPackets;     // for debugging
        FlowletInfo() = default;
        FlowletInfo(Time now, uint32_t nextHopDev) : 
            activeTime(now), createTime(now), nextHopDev(nextHopDev), nPackets(1) {};
    };

    std::map<uint32_t, NextHopItem> target2nextHop;  //target ToRID->nextHop
    std::map<uint64_t, FlowletInfo> flowletTable;  // QpKey -> Flowlet (at SrcToR)
    std::map<uint32_t, LinkInfo> devInfo;           //dev->linkInfo
    Time lastFlowletAgingTime;   //上次清除flowlet表中过期项的时间
    EventId sendProbeEvent;

    //常量
    // hula constants
    Time keepAliveThresh;        //探针老化时间
    Time probeTransmitInterval;  //转发探针的时间窗口
    Time flowletInterval;        //flowlet的区分间隔 (e.g., 100us)
    Time probeGenerationInterval;//探针的生成间隔
    Time tau;                    //计算链路利用率使用，至少是探针的生成间隔的两倍

    void generateProbe();
    void clearInvalidFlowletItem();             

   public:
    bool m_isToR;          // is ToR (leaf)
    uint32_t m_switch_id;  // switch's nodeID
    //运行过程记录
    int probeSendNum = 0;       //接收到的probe数量
    int probeReceiveNum = 0;    //发送的probe数量
    int probeAgedNum = 0;       //被老化的probe数量
    int probeUpdateHopNum = 0;  //引起最优路径更新的探针数量
};


}  // namespace ns3
