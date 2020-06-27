# DLLM Layer 2
Full controller.

## Setup
Create Virtual Environment
```bash
cd scripts
python3 -m venv ./venv
```
Start container
```bash
cd singularity
./start_container.sh
```
Install MCTS
```bash
cd /git/sferes2/exp/layer2
cp -r mcts /git
cd /git/mcts
./waf configure --prefix /workspace
./waf install
```
Compile
```bash
cd /git/sferes2/
./setup.sh
```
Visualize simulation
```bash
build/exp/layer2/example_graphic
```
Enter Virtual Environment
```bash
cd exp/layer2/scripts
source ./venv/bin/activate
```
Install dependencies
```bash
pip install -r requirements.txt
```
Visualize results
```bash
python main.py
```
