# DLLM Layer 1
Generates a repertoire for walking controllers.

## Setup
Create Virtual Environment
```bash
cd scripts
python3 -m venv ./venv
```
Start container
```bash
cd singularity
chmod +x start_container.sh
./start_container.sh
```
Compile
```bash
cd /git/sferes2/
./setup.sh
```
Enter Virtual Environment
```bash
cd exp/layer1/scripts
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

