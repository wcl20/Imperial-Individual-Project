# DLLM Layer 2
Full controller.

## Setup
Start container
```bash
cd singularity
./start_container.sh
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
Create Virtual Environment
```bash
cd exp/layer2/scripts
python3 -m venv ./venv
```
Enter Virtual Environment
```bash
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
