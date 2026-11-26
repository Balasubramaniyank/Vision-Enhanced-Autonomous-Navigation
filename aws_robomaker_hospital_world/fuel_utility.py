#!/usr/bin/env python3

import requests
import json
import logging
import time
import zipfile
import io
import os
from lxml import etree as ET

# ---------------- CONFIG ---------------- #

FUEL_URI = "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models"
DESTINATION_DIR = "fuel_models"
WAIT_BETWEEN_DOWNLOADS = 3

MODELS = [
    "XRayMachine",
    ]

# ---------------------------------------- #

logging.basicConfig(level=logging.INFO)

def download_model(model_name):
    model_dir = os.path.join(DESTINATION_DIR, model_name)
    if os.path.exists(model_dir):
        logging.info("✔ %s already exists, skipping", model_name)
        return

    logging.info("⬇ Downloading %s", model_name)

    # Default version = 1
    url = f"{FUEL_URI}/{model_name}/1/{model_name}.zip"
    r = requests.get(url)

    if r.status_code != 200:
        logging.error("✖ Failed to download %s", model_name)
        return

    with zipfile.ZipFile(io.BytesIO(r.content)) as z:
        z.extractall(model_dir)

    time.sleep(WAIT_BETWEEN_DOWNLOADS)


def create_database_file():
    database = ET.Element("database")
    ET.SubElement(database, "name").text = "Ignition Fuel Model Database"
    ET.SubElement(database, "license").text = "Creative Commons Attribution 3.0 Unported"
    models_el = ET.SubElement(database, "models")

    for m in MODELS:
        ET.SubElement(models_el, "uri").text = m

    tree = ET.ElementTree(database)
    output = os.path.join(DESTINATION_DIR, "database.config")
    tree.write(output, pretty_print=True, xml_declaration=True, encoding="utf-8")
    logging.info("✔ database.config created")


def main():
    os.makedirs(DESTINATION_DIR, exist_ok=True)

    for model in MODELS:
        download_model(model)

    create_database_file()
    logging.info("🎉 ALL MODELS DOWNLOADED SUCCESSFULLY")


if __name__ == "__main__":
    main()
