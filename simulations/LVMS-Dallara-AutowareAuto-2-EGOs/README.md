# Install ADE (one time only)

<pre>
sh OSSDC-SIM-Demos/scripts/install-ade.sh
</pre>

# Clone AutowareAuto (OSSDC version, one time only)

<pre>
cd ~/adehome
git clone --recurse-submodules https://gitlab.com/OSSDC/AutowareAuto AutowareAuto-OSSDC
cd AutowareAuto-OSSDC
git checkout OSSDC-SIM-Demos
git pull
</pre>

# Start 2 ADE envs, one for each EGO

<pre>
cd ~/adehome
pip3 install pexpect
cd AutowareAuto-OSSDC
python3 ../OSSDC-SIM-Demos/scripts/ossdc-sim-demo-start-env.py
</pre>

# Download and start OSSDC SIM

<pre>
cd ~/adehome
wget https://github.com/OSSDC/OSSDC-SIM/releases/download/OSSDC-SIM-v1.1/OSSDC-SIM-v1_1-Linux.zip
unzip OSSDC-SIM-v1_1-Linux.zip
chmod +x OSSDC-SIM-v1_1-Linux/OSSDC-SIM
cp -r OSSDC-SIM-Demos/sim-data/* OSSDC-SIM-v1_1-Linux/
cd OSSDC-SIM-v1_1-Linux
bash run-OSSDC-SIM-v1.sh
</pre>

# Select and launch (click play button) the OSSDC SIM API simulation

![image](https://user-images.githubusercontent.com/51034490/172274409-09b7d394-7cea-4cfa-9562-2e6f62dccded.png)

# Initialize the map and position the EGOs, set simulator host and bridge IPs

<pre>
cd ~/adehome
export OSSDC__SIMULATOR_HOST=127.0.0.1
export OSSDC__SIMULATOR_BRIDGE_HOST_1=127.0.0.1
export OSSDC__SIMULATOR_BRIDGE_HOST_2=127.0.0.1
python3 OSSDC-SIM-Demos/simulations/LVMS-Dallara-AutowareAuto-2-EGOs/Multi_Ego-LVMS-API.py
</pre>

# Open a new ade shell and build AutowareAuto

<pre>
cd ~/adehome
./ade --name ade0 enter
cd AutowareAuto-OSSDC
bash ../OSSDC-SIM-Demos/scripts/build-autoware-auto.sh
</pre>

# Launch AutowareAuto record/replay node

<pre>
cd ~/adehome
./ade --name ade0 enter
cd AutowareAuto-OSSDC
source /opt/AutowareAuto/setup.bash
source install/setup.bash
bash ../OSSDC-SIM-Demos/scripts/start-record-replay-autoware-auto.sh
</pre>

# Open a new shell trigger the replay

<pre>
cd ~/adehome
./ade --name ade0 enter
cd AutowareAuto-OSSDC
source /opt/AutowareAuto/setup.bash
source install/setup.bash
cp ../OSSDC-SIM-Demos/recorded-tracks/lvms-dallara-1-api-1.csv /tmp/path
bash ../OSSDC-SIM-Demos/scripts/replay-autoware-auto.sh
</pre>

The first car in the simulator should start to move and follow the recorded track

# Open a new shell to record a new track

<pre>
cd ~/adehome
./ade --name ade0 enter
cd AutowareAuto-OSSDC
source /opt/AutowareAuto/setup.bash
source install/setup.bash
bash ../OSSDC-SIM-Demos/scripts/replay-autoware-auto.sh
cp /tmp/path ../OSSDC-SIM-Demos/recorded-tracks/new_track.csv
</pre>
