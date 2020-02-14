mkdir -p simulator

wget --no-proxy https://llcad-github.llan.ll.mit.edu/TESS/tesse-icra2020-competition/releases/download/0.1.0/goseek-v0.1.0.zip  --no-check-certificate -P simulator

unzip simulator/goseek-v0.1.0.zip -d simulator

chmod +x simulator/goseek-v0.1.0.x86_64

