#!/bin/bash
set -e

python3 -m venv .venv
source .venv/bin/activate

pip install --upgrade pip
pip install numpy opencv-python-headless onnxruntime

echo "Instal·lació de les llibreries necessàries completada."
