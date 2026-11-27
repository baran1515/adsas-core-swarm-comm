# Simülasyon Ortamı

## 1. Katmanlar

| Seviye | Bileşenler | Ne zaman |
|---|---|---|
| L0 | `adsas_node` × 4, iç kinematik model | Algoritma/protokol geliştirme, CI |
| L1 | + ArduPilot SITL × 4 + `sitl_bridge.py` | Gerçek otopilot davranışı (GUIDED, kalkış, iniş) |
| L2 | + Gazebo Harmonic, `swarm_world.sdf` | Görselleştirme ve fizik |

## 2. Port Planı

| Bileşen | İHA-1 | İHA-2 | İHA-3 | İHA-4 |
|---|---|---|---|---|
| SITL örneği (`-I`) | 0 | 1 | 2 | 3 |
| MAVLink SERIAL0 (TCP) | 5760 | 5770 | 5780 | 5790 |
| Gazebo JSON FDM (UDP) | 9002 | 9012 | 9022 | 9032 |
| adsas mesh (TCP) | 47001 | 47002 | 47003 | 47004 |
| adsas köprü (UDP) | 48001 | 48002 | 48003 | 48004 |

## 3. Gazebo ile Çalıştırma

```bash
# ardupilot_gazebo eklentisi (bir kez)
sudo apt install gz-harmonic libgz-sim8-dev rapidjson-dev
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir build && cd build && cmake .. && make -j4
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=~/ardupilot_gazebo/models:~/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

# 4 model kopyası (her biri farklı FDM portu)
cd ~/adsas-core && ./sim/gazebo/make_swarm_models.sh
export GZ_SIM_RESOURCE_PATH=$PWD/sim/gazebo/models:$GZ_SIM_RESOURCE_PATH

gz sim -v4 -r sim/gazebo/swarm_world.sdf          # Terminal 1
GAZEBO=1 ./sim/launch_sitl_swarm.sh               # Terminal 2
./sim/run_sitl_demo.sh                            # Terminal 3
```

> Not: `ardupilot_gazebo` sürümleri arasında model/plugin adları değişebilir. Model kopyalama betiği,
> `iris_with_ardupilot/model.sdf` içindeki `<fdm_port_in>9002</fdm_port_in>` satırını değiştirir.

## 4. Senaryolar

| Senaryo | Komut | Beklenen |
|---|---|---|
| Düğüm kaybı | `./scripts/run_demo.sh` | İHA-4 susar → ~1 s sonra NODE_DOWN, hücreler İHA-1..3'e |
| Koordinatör kaybı | `--fail-at` İHA-1'e verilir | İHA-2 koordinatör olur, İHA-1'i düşürür |
| Jamming | `SCENARIO=jam ./scripts/run_demo.sh` | İHA-2 ISOLATED, diğerleri onu düşürür; jamming bitince İHA-2 RTL |
| Ardışık 2 arıza | İHA-3 ve İHA-4'e farklı `--fail-at` | epoch 1 → 2, kapsama korunur |

## 5. Çıktıların Değerlendirilmesi

```bash
python3 tools/plot_run.py logs -o run.png
```
Sol grafik: İHA rotaları (yerel çerçeve, m). Sağ grafik: zamana göre tamamlanan / atanan hücre sayısı;
kırmızı dikey çizgiler epoch değişimleridir (yeniden dağıtım anları).
