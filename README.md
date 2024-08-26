# NS-3 Simulator for CAVER (A LB algorithm for RDMA)
## Let's hunt less-congested paths together for better RDMA LB!

This is an NS-3 simulator for "CAVER: Enhancing RDMA Load Balancing by Hunting Less-Congested Paths". We add CAVER's components to  [ConWeave(SIGCOMM'2023)'s NS-3 simulator](https://github.com/conweave-project/conweave-ns3) to simulate RDMA load balancing performance with various algorithms.

Besides the figure of result shown in the poster, we run other experiment and the result can be shown with script  `all-to-all_visual.ipynb`.

We describe how to run this repository on docker.



## Run with Docker

#### Docker Engine
For Ubuntu, following the installation guide [here](https://docs.docker.com/engine/install/ubuntu/) and make sure to apply the necessary post-install [steps](https://docs.docker.com/engine/install/linux-postinstall/).
Eventually, you should be able to launch the `hello-world` Docker container without the `sudo` command: `docker run hello-world`.

#### 0. Prerequisites
First, you do all these:

```shell
wget https://www.nsnam.org/releases/ns-allinone-3.19.tar.bz2
tar -xvf ns-allinone-3.19.tar.bz2
cd ns-allinone-3.19
rm -rf ns-3.19
git clone https://github.com/denght23/CAVER.git ns-3.19
```

#### 1. Create a Dockerfile
Here, `ns-allinone-3.19` will be your root directory.

Create a Dockerfile at the root directory with the following:
```shell
FROM ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y gnuplot python python3 python3-pip build-essential libgtk-3-0 bzip2 wget git && rm -rf /var/lib/apt/lists/* && pip3 install install numpy matplotlib cycler
WORKDIR /root
```

Then, you do this: 
```shell
docker build -t caver-sim:version1 .
```

Once the container is built, do this from the root directory: This should build everything necessary for the simulator.
```shell
docker run -it -v $(pwd):/root caver-sim:version1 bash -c "cd ns-3.19; ./waf configure --build-profile=optimized; ./waf"
```


Do this to build a docker for the first time.
```shell
docker run -it --name caver-sim -v $(pwd):/root caver-sim:version1
```
#### 2. Run
Enter the docker 
```shell
docker start caver-sim
docker exec -it caver-sim /bin/bash
```
Run the script in the docker: 
```shell
cd ns-3.19;
./autorun.sh
```

That will run `0.1 second` simulation of the experiment for Figure 2 in the paper.

In the script, you can easily change the network load (e.g., `50%`), runtime (e.g., `0.1s`), or topology (e.g., `leaf-spine`).

This script first calls a traffic generator `./traffic_gen/traffic_gen.py` to create an input trace.
Then, it runs NS-3 simulation script `./scratch/network-load-balance.cc`.

This scirpt runs in the background, and may take servel hours to finish a simulation of 0.1 second.

#### 3.Results
The results are located at `./mix/output`.

* At `./mix/output`, several raw data is stored such as 
  * Flow Completion Time (`XXX_out_fct.txt`), 
  * PFC generation (`XXX_out_pfc.txt`), 
  * Uplink's utility (`XXX_out_uplink.txt`), 
  * Number of connections (`XXX_out_conn.txt`), 
  * Congestion Notification Packet (`XXX_out_cnp.txt`).
  * CDF of number of queues usage per egress port (`XXX_out_voq_per_dst_cdf.txt`). 
  * CDF of total queue memory overhead per switch (`XXX_out_voq_cdf.txt`). 
  
* Each run of simulation creates a repository in `./mix/output` with simulation ID (10-digit number).
* Inside the folder, you can check the simulation config `config.txt` and output log `config.log`. 
* The history of simulations will be recorded in `./mix/.history`. 

The script autorun.sh will run three different load balancing algorithms(ECMP, ConWeave and CAVER) with the same topology and traffic, each method will be allocated a simulation ID(10-digit number), and the newest simulation ID will be added to the end of the `./mix/.history` file. The structs of `./mix/.history` are as follows
```shell
05/20/24,176527761,1,0,1000,4,16,64,400,1,0,0,0,fat_k_4_OS1,100,AliStorage2019,60,0.1
./waf --run 'scratch/network-load-balance /root/ns-3.19/mix/output/176527761/config.txt' > /root/ns-3.19/mix/output/176527761/config.log 2>&1
./waf --run 'scratch/network-load-balance' --command-template='gdb --args %s /root/ns-3.19/mix/output/176527761/config.txt'

05/20/24,918924462,1,9,1000,4,16,64,600,1,0,0,0,fat_k_4_OS1,100,AliStorage2019,60,0.1
./waf --run 'scratch/network-load-balance /root/ns-3.19/mix/output/918924462/config.txt' > /root/ns-3.19/mix/output/918924462/config.log 2>&1
./waf --run 'scratch/network-load-balance' --command-template='gdb --args %s /root/ns-3.19/mix/output/918924462/config.txt'

05/20/24,744923884,1,10,1000,4,16,64,400,1,0,0,0,fat_k_4_OS1,100,AliStorage2019,60,0.1
./waf --run 'scratch/network-load-balance /root/ns-3.19/mix/output/744923884/config.txt' > /root/ns-3.19/mix/output/744923884/config.log 2>&1
./waf --run 'scratch/network-load-balance' --command-template='gdb --args %s /root/ns-3.19/mix/output/744923884/config.txt'
```
The meaning of each entry's first 4 values is as follows: time, simulation id, cc_mode(1:DCQCN), load_mode(0:ECMP, 9:ConWeave, 10:CAVER).

#### 4. Plot
You can run experiment following the previous steps.

But as each experiment may take several hours, you can download result from the below link and save them to the `./mix/output` folder (**we recommand this way**):

[results of CAVER](https://cloud.tsinghua.edu.cn/d/b55b00eb0ac240db983b/) 


You can easily plot the results using the following script `./show/all-to-all_visual.py` to generate picture as Fig 2 in the posters:

Step 1: 
After run the 'autorun.sh' script, find each algorithm 's Simulation ID in the last three entries of `./mix/.history` file.

Step 2:
Add each algorithm's simulation ID to the `./show/all-to-all_visual.py` as the following example (3-tier fat_tree, k = 4, load = 49%).
```shell
fecmp_id = 497547479
conweave_id = 907715927
CAVER_id = 545564951
```
The figure will be save as `plot.pdf` and `plot.png`

We also provide a `./show/all-to-all_visual.ipynb` with the same method.

And the results of different traffic load are stored in this script.

The figure will be save as `plot.pdf` and `plot.png`


##### Topology
To evaluate on topology of a fat-tree (K=4) with bond, you can simply change the `TOPOLOGY` variable in `autorun.sh` to `fat_k_4_OS1`:
```shell
TOPOLOGY="fat_k_4_OS1"
```

##### Clean up
To clean all data of previous simulation results, you can run the command:
```shell
./cleanup.sh
```

#### CAVER Parameters
We include CAVER's parameter values into `./run.py` based on flow control model and topology.  


### Simulator Structure
Most implementations of network load balancing are located in the directory `./src/point-to-point/model`.

* `switch-node.h/cc`: Switching logic that includes a default multi-path routing protocol (e.g., ECMP) .
* `switch-mmu.h/cc`: Ingress/egress admission control and PFC.
* `dv-routing.cc`: CAVER routing protocol.
* `conweave-routing.h/cc`: ConWeave routing protocol, the original code doesn't support the bond scenario, we modifiy the code by select each flow's Src ToR and Dst ToR following a round-robin policy.
* `conweave-voq.h/cc`: ConWeave in-network reordering buffer.
* `settings.h/cc`: Global variables for logging and debugging.
* `rdma-hw.h/cc`: RDMA-enable NIC behavior model.

<b> RNIC behavior model to out-of-order packet arrival </b>
As disussed in the paper, we observe that RNIC reacts to even a single out-of-order packet sensitively by sending CNP packet.
However, existing RDMA-NS3 simulator (HPCC, DCQCN, TLT-RDMA, etc) did not account for this.
In this simulator, we implemented that behavior in `rdma-hw.cc`.


