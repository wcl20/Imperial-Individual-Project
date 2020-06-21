# DLLM Layer 1
Generates a repertoire for walking controllers.

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
cd exp/layer1/scripts
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

