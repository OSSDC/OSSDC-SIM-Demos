# Install ADE (one time only)

sh OSSDC-SIM-Demos/scripts/install-ade.sh

# Clone AutowareAuto (OSSDC version, one time only)

cd ${HOME}/adehome

git clone --recurse-submodules https://gitlab.com/OSSDC/AutowareAuto AutowareAuto-OSSDC

cd AutowareAuto-OSSDC

git checkout OSSDC-SIM-Demos

git pull

# Start 2 ADE envs, one for each EGO

cd ${HOME}/adehome

pip3 install pexpect

cd AutowareAuto-OSSDC

python3 ../OSSDC-SIM-Demos/scripts/ossdc-sim-demo-start-env.py

# Download and start OSSDC SIM

cd ${HOME}/adehome

wget https://github.com/OSSDC/OSSDC-SIM/releases/download/2021.3-OSSDC-SIM-1/OSSDC-SIM-v1-Linux.zip

unzip OSSDC-SIM-v1-Linux.zip

chmod +x OSSDC-SIM-v1-Linux/OSSDC-SIM

cp -r OSSDC-SIM-Demos/sim-data/* OSSDC-SIM-v1-Linux/

sh OSSDC-SIM-v1-Linux/run-OSSDC-SIM-v1.sh

# Select and launch the OSSDC SIM API simulation

# Initialize the map and position the EGOs

cd ${HOME}/adehome

python3 OSSDC-SIM-Demos/simulations/LVMS-Dallara-AutowareAuto-2-EGOs/Multi_Ego-LVMS-API.py

# Open a new ade shell and build AutowareAuto

cd ${HOME}/adehome

ade --name ade0 enter

cd AutowareAuto-OSSDC

bash ../OSSDC-SIM-Demos/scripts/build-autoware-auto.sh

# Launch AutowareAuto record/replay node

cd ${HOME}/adehome

ade --name ade0 enter

cd AutowareAuto-OSSDC

source install/setup.bash

bash ../OSSDC-SIM-Demos/scripts/start-record-replay-autoware-auto.sh

# Open a new shell trigger the replay

cd ${HOME}/adehome

ade --name ade0 enter

cd AutowareAuto-OSSDC

source install/setup.bash

cp ../OSSDC-SIM-Demos/recorded-tracks/lvms-dallara-1-api-1.csv /tmp/path

bash ../OSSDC-SIM-Demos/scripts/replay-autoware-auto.sh

The first car in the simulator should start to move and follow the recorded track
