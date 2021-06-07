/////////////////////////////////////////////////////////
/// 191 IT 122 [ Ishwar Chandra Mandal ]
/// 191 IT 132 [ Navaneeth P           ]
/// 191 IT 222 [ K Prajwal             ]
/// 191 IT 255 [ Tharun K              ]
/////////////////////////////////////////////////////////

#include "ns3/applications-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include <bits/stdc++.h>
#include <cfloat>
#include <iostream>
#include <random>
#include <sstream>

#define k 3
#define n 10

using namespace ns3;
using namespace std;

struct Point {
    double x, y;
    int cluster;
    uint32_t nodeId1;
    uint32_t nodeId2;
    int nodeType;
    Ipv4Address nodeIp1;
    Ipv4Address nodeIp2;
    double minDist;

    Point(double x, double y, int type, uint32_t node1, uint32_t node2 = -1)
        : x(x)
        , y(y)
        , cluster(-1)
        , nodeId1(node1)
        , nodeId2(node2)
        , nodeType(type)
        , minDist(__DBL_MAX__)
    {
    }

    double distance(Point p)
    {
        return (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
    }
};

void kMeansClustering(vector<Point>* points)
{
    vector<Point> centroids;
    srand(time(0));
    for (int i = 0; i < k; ++i) {
        for (vector<Point>::iterator x = points->begin(); x != points->end(); ++x) {
            if (x->nodeType == 1 && x->cluster == -1) {
                centroids.push_back(*x);
            }
        }
        for (vector<Point>::iterator c = begin(centroids); c != end(centroids); ++c) {
            int clusterId = c - begin(centroids);
            for (vector<Point>::iterator it = points->begin(); it != points->end(); ++it) {
                Point p = *it;
                double dist = c->distance(p);
                if (dist < p.minDist) {
                    p.minDist = dist;
                    p.cluster = clusterId;
                }
                *it = p;
            }
        }
    }
    vector<int> nPoints;
    vector<double> sumX, sumY;

    for (int j = 0; j < k; ++j) {
        nPoints.push_back(0);
        sumX.push_back(0.0);
        sumY.push_back(0.0);
    }

    for (vector<Point>::iterator it = points->begin(); it != points->end(); ++it) {
        int clusterId = it->cluster;
        nPoints[clusterId] += 1;
        sumX[clusterId] += it->x;
        sumY[clusterId] += it->y;

        it->minDist = __DBL_MAX__;
    }

    for (vector<Point>::iterator c = begin(centroids); c != end(centroids); ++c) {
        int clusterId = c - begin(centroids);
        c->x = sumX[clusterId] / nPoints[clusterId];
        c->y = sumY[clusterId] / nPoints[clusterId];
    }

    cout << "shard \t node_id \t node_ip_address \t x \t y \t z \t nodeType" << endl;

    sort(begin(*points), end(*points), [](Point const& p1, Point const& p2) {
        return p1.cluster < p2.cluster;
    });

    for (vector<Point>::iterator it = points->begin(); it != points->end(); ++it) {
        cout << it->cluster << " \t\t " << it->nodeId1 << " \t\t " << it->nodeIp1 << " \t " << it->x << " \t " << it->y << " \t " << 0 << " \t " << (it->nodeType == 1 ? "CU" : "UE") << endl;
        if (it->nodeType == 0) {
            cout << it->cluster << " \t\t " << it->nodeId2 << " \t\t " << it->nodeIp2 << " \t " << it->x << " \t " << it->y << " \t " << 0 << " \t " << (it->nodeType == 1 ? "CU" : "UE") << endl;
        }
    }

    cout << "\n\n";
}

int64_t printTime = 2500000000;

void monitor(std::string context, uint16_t cellId, uint16_t rnti, double rsrp, double sinr, uint8_t componentCarrierId)
{
    int64_t time = Simulator::Now().GetNanoSeconds();

    for (auto i = printTime; i <= time; i += 1000000000) {
        if (time >= i && time <= i + 1000000) {
            cout << time / (double)1e9 << "\t";
            cout << sinr << "\t";
            cout << "\n";
        }
    }
}

NS_LOG_COMPONENT_DEFINE("LteSlInCovrgCommMode1");

int main(int argc, char* argv[])
{
    Time simTime = Seconds(6);

    // Configure the scheduler
    Config::SetDefault("ns3::RrSlFfMacScheduler::Itrp", UintegerValue(0));

    //The number of RBs allocated per UE for Sidelink
    Config::SetDefault("ns3::RrSlFfMacScheduler::SlGrantSize", UintegerValue(5));

    //Set the frequency
    Config::SetDefault("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue(100));
    Config::SetDefault("ns3::LteUeNetDevice::DlEarfcn", UintegerValue(100));
    Config::SetDefault("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue(18100));
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(50));
    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(50));

    // Set error models
    Config::SetDefault("ns3::LteSpectrumPhy::SlCtrlErrorModelEnabled", BooleanValue(true));
    Config::SetDefault("ns3::LteSpectrumPhy::SlDataErrorModelEnabled", BooleanValue(true));
    Config::SetDefault("ns3::LteSpectrumPhy::DropRbOnCollisionEnabled", BooleanValue(false));

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    //Set the UEs power in dBm
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(23.0));

    //Set the eNBs power in dBm
    //cHJhandhbG5hdmFuZWV0aHRoYXJ1bmlzaHdhcg==
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(30.0));

    //Sidelink bearers activation time
    Time slBearersActivationTime = Seconds(2.0);

    //Create the helpers
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();

    //Create and set the EPC helper
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    ////Create Sidelink helper and set lteHelper (ishwar)
    Ptr<LteSidelinkHelper> proseHelper = CreateObject<LteSidelinkHelper>();
    proseHelper->SetLteHelper(lteHelper);

    //Set pathloss model
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::Cost231PropagationLossModel"));

    //Enable Sidelink
    lteHelper->SetAttribute("UseSidelink", BooleanValue(true));

    //Sidelink Round Robin scheduler (it222)
    lteHelper->SetSchedulerType("ns3::RrSlFfMacScheduler");

    //Create nodes (eNb + UEs)
    NodeContainer enbNode;
    enbNode.Create(1);

    NodeContainer ueNodes;
    NodeContainer ueClients;
    NodeContainer ueServers;
    std::vector<NodeContainer> uePairs(n);

    for (uint32_t i = 0; i < n; ++i) {
        NodeContainer pair;
        pair.Create(2);

        ueClients.Add(pair.Get(0));
        ueServers.Add(pair.Get(1));

        ueNodes.Add(pair);

        uePairs[i].Add(pair);
    }

    NodeContainer cuNodes;
    cuNodes.Create(k);

    //Position of the nodes
    Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator>();
    positionAllocEnb->Add(Vector(0.0, 0.0, 0.0));

    random_device seed;
    mt19937 gen(seed());
    uniform_real_distribution<float> dist(0.0, 50.0);

    int x1, y1, x2, y2;
    vector<Point> points;

    Ptr<ListPositionAllocator> positionAllocUe[n];
    for (int i = 0; i < n; i++) {
        x1 = dist(gen);
        y1 = dist(gen);

        x2 = x1 + 10;
        y2 = y1 + 10;

        points.push_back(Point((x1 + x2) / 2, (y1 + y2) / 2, 0, uePairs[i].Get(0)->GetId(), uePairs[i].Get(1)->GetId()));

        positionAllocUe[i] = CreateObject<ListPositionAllocator>();
        positionAllocUe[i]->Add(Vector(x1, y1, 0.0));
        positionAllocUe[i]->Add(Vector(x2, y2, 0.0));
    }

    Ptr<ListPositionAllocator> positionAllocCu[k];
    for (int i = 0; i < k; i++) {
        x1 = dist(gen);
        y1 = dist(gen);
        points.push_back(Point(x1, y1, 1, cuNodes.Get(i)->GetId()));
        positionAllocCu[i] = CreateObject<ListPositionAllocator>();
        positionAllocCu[i]->Add(Vector(x1, y1, 0.0));
    }

    //Install mobility
    MobilityHelper mobilityeNodeB;
    mobilityeNodeB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityeNodeB.SetPositionAllocator(positionAllocEnb);
    mobilityeNodeB.Install(enbNode);

    MobilityHelper mobilityUe[n];
    for (int i = 0; i < n; i++) {
        mobilityUe[i].SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobilityUe[i].SetPositionAllocator(positionAllocUe[i]);
        mobilityUe[i].Install(uePairs[i]);
    }

    MobilityHelper mobilityCu[k];
    for (int i = 0; i < k; i++) {
        mobilityCu[i].SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobilityCu[i].SetPositionAllocator(positionAllocCu[i]);
        mobilityCu[i].Install(cuNodes.Get(i));
    }

    //Install LTE devices to the nodes and fix the random number stream (it132)
    int64_t randomStream = 1;

    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNode);
    randomStream += lteHelper->AssignStreams(enbDevs, randomStream);

    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);
    randomStream += lteHelper->AssignStreams(ueDevs, randomStream);

    NetDeviceContainer cuDevs = lteHelper->InstallUeDevice(cuNodes);
    randomStream += lteHelper->AssignStreams(cuDevs, randomStream);

    //Configure Sidelink Pool
    Ptr<LteSlEnbRrc> enbSidelinkConfiguration = CreateObject<LteSlEnbRrc>();
    enbSidelinkConfiguration->SetSlEnabled(true);

    //Preconfigure pool for the group
    LteRrcSap::SlCommTxResourcesSetup pool;

    pool.setup = LteRrcSap::SlCommTxResourcesSetup::SCHEDULED;

    //BSR timers (it255)
    pool.scheduled.macMainConfig.periodicBsrTimer.period = LteRrcSap::PeriodicBsrTimer::sf16;
    pool.scheduled.macMainConfig.retxBsrTimer.period = LteRrcSap::RetxBsrTimer::sf640;

    //MCS
    pool.scheduled.haveMcs = true;
    pool.scheduled.mcs = 16;

    //resource pool
    LteSlResourcePoolFactory pfactory;
    pfactory.SetHaveUeSelectedResourceConfig(false); //since we want eNB to schedule

    //Control
    pfactory.SetControlPeriod("sf40");
    pfactory.SetControlBitmap(0x00000000FF); //8 subframes for PSCCH
    pfactory.SetControlOffset(0);
    pfactory.SetControlPrbNum(22);
    pfactory.SetControlPrbStart(0);
    pfactory.SetControlPrbEnd(49);

    //Data: The ns3::RrSlFfMacScheduler is responsible to handle the parameters (navaneeth)

    pool.scheduled.commTxConfig = pfactory.CreatePool();

    uint32_t groupL2Address = 255;

    enbSidelinkConfiguration->AddPreconfiguredDedicatedPool(groupL2Address, pool);
    lteHelper->InstallSidelinkConfiguration(enbDevs, enbSidelinkConfiguration);

    //pre-configuration for the UEs
    Ptr<LteSlUeRrc> ueSidelinkConfiguration = CreateObject<LteSlUeRrc>();
    ueSidelinkConfiguration->SetSlEnabled(false);
    LteRrcSap::SlPreconfiguration preconfiguration;
    ueSidelinkConfiguration->SetSlPreconfiguration(preconfiguration);
    lteHelper->InstallSidelinkConfiguration(ueDevs, ueSidelinkConfiguration);

    NodeContainer allNodes;
    allNodes.Add(ueNodes);
    allNodes.Add(cuNodes);

    NetDeviceContainer allDevs;
    allDevs.Add(ueDevs);
    allDevs.Add(cuDevs);

    InternetStackHelper internet;
    internet.Install(allNodes);
    Ipv4Address groupAddress4("225.0.0.0"); //use multicast address as destination

    //Install the IP stack on the UEs and assign IP address (prajwal)
    Ipv4InterfaceContainer allIpIface;
    allIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(allDevs));

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
        Ptr<Node> allNode = allNodes.Get(i);

        // Set the default gateway for the UEIpv4Address
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(allNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    Address remoteAddress = InetSocketAddress(groupAddress4, 8000);
    Address localAddress = InetSocketAddress(Ipv4Address::GetAny(), 8000);
    Ptr<LteSlTft> tft = Create<LteSlTft>(LteSlTft::BIDIRECTIONAL, groupAddress4, groupL2Address);

    for (int i = 0; i < n; i++) {
        points[i].nodeIp1 = uePairs[i].Get(0)->GetObject<Ipv4L3Protocol>()->GetAddress(1, 0).GetLocal();
        points[i].nodeIp2 = uePairs[i].Get(1)->GetObject<Ipv4L3Protocol>()->GetAddress(1, 0).GetLocal();
    }

    for (int i = 0; i < k; i++) {
        points[n + i].nodeIp1 = cuNodes.Get(i)->GetObject<Ipv4L3Protocol>()->GetAddress(1, 0).GetLocal();
    }

    //Attach each UE to the best available eNB
    lteHelper->Attach(ueDevs);
    lteHelper->Attach(cuDevs);

    kMeansClustering(&points);

    ///*** Configure applications (tharun) ***///

    //Set Application in the UEs
    OnOffHelper sidelinkClient("ns3::UdpSocketFactory", remoteAddress);
    sidelinkClient.SetConstantRate(DataRate("16KB/s"), 200);
    for (int shard = 0; shard < k; ++shard) {
        int time = 2;

        for (auto&& uePair : uePairs) {
            for (auto&& point : points) {
                if (point.cluster == shard && uePair.Get(0)->GetId() == point.nodeId1) {
                    ApplicationContainer clientApp = sidelinkClient.Install(uePair.Get(0));
                    clientApp.Start(Seconds(time));
                    clientApp.Stop(simTime);
                    time += 1;
                }
            }
        }
    }

    ApplicationContainer serverApps;
    PacketSinkHelper sidelinkSink("ns3::UdpSocketFactory", localAddress);
    serverApps = sidelinkSink.Install(ueServers);
    serverApps.Start(Seconds(2.0));

    proseHelper->ActivateSidelinkBearer(slBearersActivationTime, ueDevs, tft);

    ///*** End of application configuration (it122) ***//

    vector<vector<Point>> shards(k);
    for (auto&& point : points) {
        if (point.nodeType == 0) {
            shards[point.cluster].push_back(point);
        }
    }

    cout << "\nSINR\n"
         << "\n";
    cout << "Time(s)"
         << "\t"
         << "sinr"
         << "\n";

    for (auto&& shard : shards) {
        for (auto&& point : shard) {
            Config::Connect("/NodeList/" + to_string(point.nodeId2) + "/DeviceList/0/$ns3::LteUeNetDevice/ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr", MakeCallback(&monitor));
        }
    }

    //Enabling Sidelink traces...
    lteHelper->EnableSidelinkTraces();

    Simulator::Stop(simTime);

    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
