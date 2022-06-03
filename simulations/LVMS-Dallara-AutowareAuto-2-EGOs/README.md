# Install ADE (one time only)

sh OSSDC-SIM-Demos/scripts/install-ade.sh

# Clone AutowareAuto (OSSDC version, one time only)

git clone --recurse-submodules https://gitlab.com/OSSDC/AutowareAuto
git checkout OSSDC-SIM-Demos
git pull

# Start 2 ADE envs, one for each EGO

pip3 install pexpect
python3 ../OSSDC-SIM-Demos/scripts/ossdc-sim-demo-start-env.py


# Download and start OSSDC SIM

cd ~/adehome
wget https://github.com/OSSDC/OSSDC-SIM/releases/download/2021.3-OSSDC-SIM-1/OSSDC-SIM-v1-Linux.zip
unzip OSSDC-SIM-v1-Linux.zip
cp -r sim-data/* OSSDC-SIM-v1-Linux/

sh OSSDC-SIM-v1-Linux/run-OSSDC-SIM-v1.sh

# Select and launch the OSSDC SIM API simulation

# Initialize the map and position the EGOs

python3 OSSDC-SIM-Demos/simulations/LVMS-Dallara-AutowareAuto-2-EGOs/Multi_Ego-LVMS-API.py

# Open a new ade shell and build AutowareAuto

ade --name ade0 enter
sh OSSDC-SIM-Demos/scripts/build-autoware-auto.sh

# Launch AutowareAuto record/replay node

sh OSSDC-SIM-Demos/scripts/start-record-replay-autoware-auto.sh

# Open a new shell trigger the replay

ade --name ade0 enter
source AutowareAuto-OSSDC/install/setup.bash
cp OSSDC-SIM-Demos/recorded-tracks/lvms-dallara-1-api-1.csv /tmp/path
sh OSSDC-SIM-Demos/scripts/replay-autoware-auto.sh

The first car in the simulator should start to move and follow the recorded track