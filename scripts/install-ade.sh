python3 -m pip install --upgrade pip

# Install docker

sudo apt install docker.io -y

sudo usermod -aG docker $USER
newgrp docker

#check docker is working
docker run hello-world

# add networks for AutowareAuto EGO 1 and 2
docker network create ade0
docker network create ade1

# install ADE

mkdir ~/adehome
cd ~/adehome
touch .adehome

wget https://gitlab.com/ApexAI/ade-cli/-/jobs/1341322851/artifacts/raw/dist/ade+x86_64
mv ade+x86_64 ade
chmod +x ade
cp ade ~/.local/bin
which ~/adehome/ade

~/adehome/ade update-cli
