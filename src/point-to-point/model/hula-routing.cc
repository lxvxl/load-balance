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

#include "hula-routing.h"

#include "assert.h"
#include "ns3/assert.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-header.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/settings.h"
#include "ns3/simulator.h"
#include "ns3/header.h"
#include "ns3/int-header.h"

NS_LOG_COMPONENT_DEFINE("HulaRouting");

namespace ns3 {
    uint32_t HulaRouting::nFlowletTimeout = 0;
    std::vector<HulaRouting*> HulaRouting::hulaModules;

    /*----- Hula-Route ------*/
    HulaRouting::HulaRouting() {
        m_isToR = false;
        m_switch_id = (uint32_t)-1;
        // set constan
        lastFlowletAgingTime = Seconds(0);
        HulaRouting::hulaModules.push_back(this);
    }

    void HulaRouting::active(int time) {
        sendProbeEvent = Simulator::Schedule(Seconds(time), &HulaRouting::generateProbe, this);
    }

    // it defines flowlet's 64bit key (order does not matter)
    uint64_t HulaRouting::GetQpKey(uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg) {
        return ((uint64_t)dip << 32) | ((uint64_t)sport << 16) | (uint64_t)pg | (uint64_t)dport;
    }

    TypeId HulaRouting::GetTypeId(void) {
        static TypeId tid =
            TypeId("ns3::CongaRouting").SetParent<Object>().AddConstructor<HulaRouting>();

        return tid;
    }

    /** CALLBACK: callback functions  */
    void HulaRouting::DoSwitchSend(Ptr<Packet> p, CustomHeader& ch, uint32_t outDev, uint32_t qIndex) {
        m_switchSendCallback(p, ch, outDev, qIndex);
    }

    void HulaRouting::DoSwitchSendToDev(Ptr<Packet> p, CustomHeader& ch) {
        m_switchSendToDevCallback(p, ch);
    }

    void HulaRouting::DoSwitchSendHulaProbe(uint32_t dev, uint32_t torID, uint8_t minUtil) {
        m_switchSendHulaProbeCallback(dev, torID, minUtil);
        probeSendNum++;
    }

    void HulaRouting::SetSwitchSendCallback(SwitchSendCallback switchSendCallback) {
        m_switchSendCallback = switchSendCallback;
    }

    void HulaRouting::SetSwitchSendToDevCallback(SwitchSendToDevCallback switchSendToDevCallback) {
        m_switchSendToDevCallback = switchSendToDevCallback;
    }

    void HulaRouting::SetSwitchSendHulaProbeCallback(SwitchSendHulaProbeCallback switchSendHulaProbeCallback) {
        m_switchSendHulaProbeCallback = switchSendHulaProbeCallback;
    }

    void HulaRouting::SetSwitchInfo(bool isToR, uint32_t switch_id) {
        m_isToR = isToR;
        m_switch_id = switch_id;
    }

    void HulaRouting::SetLinkCapacity(uint32_t outPort, uint64_t bitRate) {
        auto it = devInfo.find(outPort);
        if (it != devInfo.end()) {
            // already exists, then check matching
            NS_ASSERT_MSG(it->second.maxBitRate == bitRate,
                        "bitrate already exists, but inconsistent with new input");
        } else {
            devInfo[outPort].maxBitRate = bitRate;
        }
    }

    void HulaRouting::SetConstants(Time keepAliveThresh,
                            Time probeTransmitInterval,
                            Time flowletInterval,
                            Time probeGenerationInterval,
                            Time tau) {
        this->keepAliveThresh = keepAliveThresh;
        this->probeTransmitInterval = probeTransmitInterval;
        this->flowletInterval = flowletInterval;
        this->probeGenerationInterval = probeGenerationInterval;
        this->tau = tau;
    }

    /* CongaRouting's main function */
    void HulaRouting::RouteInput(Ptr<Packet> p, CustomHeader ch) {
    // Packet arrival time
        Time now = Simulator::Now();

        if (ch.l3Prot != 0x11) {
            DoSwitchSendToDev(p, ch);
            return;
        }
        assert(ch.l3Prot == 0x11 && "Only supports UDP data packets");

        // get srcToRId, dstToRId
        assert(Settings::hostIp2SwitchId.find(ch.sip) !=
            Settings::hostIp2SwitchId.end());  // Misconfig of Settings::hostIp2SwitchId - sip"
        assert(Settings::hostIp2SwitchId.find(ch.dip) !=
            Settings::hostIp2SwitchId.end());  // Misconfig of Settings::hostIp2SwitchId - dip"
        uint32_t srcToRId = Settings::hostIp2SwitchId[ch.sip];
        uint32_t dstToRId = Settings::hostIp2SwitchId[ch.dip];

        /** FILTER: Quickly filter intra-pod traffic */
        if (srcToRId == dstToRId) {  // do normal routing (only one path)
            DoSwitchSendToDev(p, ch);
            return;
        }

        // 如果当前表项还没有初始化，就使用ECMP
        if (target2nextHop.find(dstToRId) == target2nextHop.end()) {
            DoSwitchSendToDev(p, ch);
            if (Simulator::Now() > Seconds(2.0001)) {
                //std::cout<<Simulator::Now()<<"路由表尚未初始化\n";
            }
            return;
        }

        // it should be not in the same pod
        assert(srcToRId != dstToRId && "Should not be in the same pod");

        // get QpKey to find flowlet
        uint64_t qpkey = GetQpKey(ch.dip, ch.udp.sport, ch.udp.dport, ch.udp.pg);
        if (flowletTable.find(qpkey) != flowletTable.end()              //如果已经有了这个flowlet
            && now - flowletTable[qpkey].activeTime < flowletInterval) {
            flowletTable[qpkey].activeTime = now;
            flowletTable[qpkey].nPackets++;
            DoSwitchSend(p, ch, flowletTable[qpkey].nextHopDev, ch.udp.pg);
        } else {
            //printf("another flow\n");
            if (flowletTable.find(qpkey) != flowletTable.end()) { //分片
                HulaRouting::nFlowletTimeout++;
            }
            flowletTable[qpkey] = FlowletInfo(now, target2nextHop[dstToRId].nextHopDev);
            DoSwitchSend(p, ch, flowletTable[qpkey].nextHopDev, ch.udp.pg);
        }
        if (now - lastFlowletAgingTime > flowletInterval * 3) {
            clearInvalidFlowletItem();
        }
    }

    void HulaRouting::processProbe(uint32_t inDev, Ptr<Packet> p, CustomHeader ch) {
        assert(ch.l3Prot == 0xFB);
        uint32_t torID = ch.hula.torID;
        uint8_t  util = ch.hula.minUtil;
        uint8_t  minUtil  = std::max(util, (uint8_t)(devInfo[inDev].curUtil / (devInfo[inDev].maxBitRate * tau.GetSeconds()) * 256));
        Time now = Simulator::Now();
        if (m_switch_id == 128) {
            //printf("received a probe, indev=%d, torid=%d, minUtil=%d\n", inDev, torID, util);
        }
        if (target2nextHop[torID].nextHopDev == inDev                       //如果输入源与当前下一跳一致
            || now - target2nextHop[torID].lastUpdateTime > keepAliveThresh //如果当前下一条已经老化
            || target2nextHop[torID].pathUtil > minUtil) {                  //如果当前链路状态大于之前的
            if (now - target2nextHop[torID].lastUpdateTime > keepAliveThresh) {
                probeAgedNum++;
            }
            if (target2nextHop[torID].pathUtil > minUtil && inDev != target2nextHop[torID].nextHopDev) {
                probeUpdateHopNum++;
            }
            target2nextHop[torID].nextHopDev = inDev;
            target2nextHop[torID].pathUtil = minUtil;
            target2nextHop[torID].lastUpdateTime = now;
        }
        ch.hula.minUtil = minUtil;
        if (now - target2nextHop[torID].lastProbeSendTime > probeTransmitInterval && !m_isToR) { //如果需要转发
            for (auto dev : downLayerDevs) { //先将原探针转发到下层
                if (dev == inDev) {
                    continue;
                }
                DoSwitchSendHulaProbe(dev, torID, minUtil);
            }
            if (downLayerDevs.find(inDev) != downLayerDevs.end()) { //如果原探针是从下层传过来的，需要转发到上层
                for (auto dev : upLayerDevs) {
                    DoSwitchSendHulaProbe(dev, torID, minUtil);
                }
            }
            target2nextHop[torID].lastProbeSendTime = now;
        }
        probeReceiveNum++;
    }

    void HulaRouting::clearInvalidFlowletItem() {
        Time now = Simulator::Now();
        for (auto it = flowletTable.begin(); it != flowletTable.end();) {
            if (now - it->second.activeTime > flowletInterval) {
                it = flowletTable.erase(it);
            } else {
                ++it;
            }
        }
        lastFlowletAgingTime = now;
    }

    void HulaRouting::updateLink(uint32_t dev, uint32_t packetSize) {
        Time now = Simulator::Now();
        devInfo[dev].curUtil = packetSize + (devInfo[dev].curUtil * (1 - (now - devInfo[dev].lastUpdateTime) / tau)).GetDouble();
        devInfo[dev].lastUpdateTime = now;
    }

    void HulaRouting::generateProbe() {
        for (auto dev : upLayerDevs) {
            //std::cout<<Simulator::Now()<<m_switch_id<<"send a probe on "<<dev<<std::endl;
            DoSwitchSendHulaProbe(dev, m_switch_id, 0);
        }
        sendProbeEvent = Simulator::Schedule(probeGenerationInterval, &HulaRouting::generateProbe, this);
    }

    void HulaRouting::Print() {
        std::cout<<"===============Switch "<<m_switch_id<<"====================\n";
        for (auto& pair : target2nextHop) {
            printf("dst %d: %d %d\n", pair.first, pair.second.nextHopDev, (int)pair.second.pathUtil);
        }
        for (auto& pair : devInfo) {
            printf("link %d: %ld\n", pair.first, pair.second.curUtil);
        }
    }
}  // namespace ns3