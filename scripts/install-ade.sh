mkdir ~/adehome
cd ~/adehome
touch .adehome

wget https://gitlab.com/ApexAI/ade-cli/-/jobs/1341322851/artifacts/raw/dist/ade+x86_64
mv ade+x86_64 ade
chmod +x ade
cp ade ~/.local/bin
which ade

ade update-cli