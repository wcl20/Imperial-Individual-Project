# DLLM Layer 0
Generates a repertoire for each leg.

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
Create Virtual Environment
```bash
cd exp/layer0/scripts
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
