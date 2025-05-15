#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-05-15
# modified: 2025-05-15
#

import os
import subprocess
import shutil
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_log = Logger("generate-docs", Level.INFO)

# paths for conf.py and index.rst
CONF_PATH         = "docs/source/conf.py"
INDEX_PATH        = "docs/source/index.rst"
CONF_BACKUP_PATH  = f"{CONF_PATH}.backup"
INDEX_BACKUP_PATH = f"{INDEX_PATH}.backup"
SOURCE_DIR        = "docs/source/"
BUILD_DIR         = "docs/_build/html"

def backup_file(file_path, backup_path):
    '''
    Function to back up a file.
    '''
    if os.path.exists(file_path):
        _log.info("backing up: {}".format(file_path))
        shutil.move(file_path, backup_path)

def restore_file(file_path, backup_path):
    '''
    Function to restore a file from backup.
    '''
    if os.path.exists(backup_path):
        _log.info("restoring: {}".format(file_path))
        shutil.move(backup_path, file_path)

def clean_generated_files():
    '''
    Clean the generated files in the docs/source and docs/_static/_templates.
    '''
    _log.info("cleaning up old .rst files and other generated files…")
    for root, dirs, files in os.walk(SOURCE_DIR):
        for file in files:
            if file.endswith(".rst") and file != "modules.rst":
                os.remove(os.path.join(root, file))
#   shutil.rmtree(os.path.join(SOURCE_DIR, "_static"), ignore_errors=True)
    shutil.rmtree(os.path.join(SOURCE_DIR, "_templates"), ignore_errors=True)

def generate_rst_for_dir(directory):
    '''
    Function to generate .rst files using sphinx-apidoc.
    '''
    _log.info("generating .rst files for {} directory…".format(directory))
    subprocess.run(["sphinx-apidoc", "-o", SOURCE_DIR, directory, "--force"])

def generate_root_level_rst():
    '''
    Function to generate .rst files for root-level files.
    '''
    _log.info("including root-level Python files in .rst generation…")
    with open(os.path.join(SOURCE_DIR, "krzos.rst"), 'w') as f:
        f.write("KRZOS Module\n============\n\n")
        f.write(".. automodule:: krzos\n   :members:\n   :undoc-members:\n   :show-inheritance:\n   :no-index:\n")
        
    with open(os.path.join(SOURCE_DIR, "krzosd.rst"), 'w') as f:
        f.write("KRZOSD Module\n=============\n\n")
        f.write(".. automodule:: krzosd\n   :members:\n   :undoc-members:\n   :show-inheritance:\n   :no-index:\n")

def build_documentation():
    '''
    Function to build the documentation.
    '''
    _log.info("building documentation…")
    subprocess.run(["sphinx-build", "-b", "html", SOURCE_DIR, BUILD_DIR])

# Main function to perform the entire workflow
def main():
    # backup conf.py and index.rst
    backup_file(CONF_PATH, CONF_BACKUP_PATH)
    backup_file(INDEX_PATH, INDEX_BACKUP_PATH)
    # clean generated files
    clean_generated_files()
    # restore conf.py and index.rst
    restore_file(CONF_PATH, CONF_BACKUP_PATH)
    restore_file(INDEX_PATH, INDEX_BACKUP_PATH)
    # generate .rst files for directories
    generate_rst_for_dir("behave")
    generate_rst_for_dir("core")
    generate_rst_for_dir("hardware")
    # generate .rst files for root-level files
    generate_root_level_rst()
    # build the documentation
    build_documentation()
    _log.info("documentation generated in: " + Fore.GREEN + "{}".format(BUILD_DIR))

if __name__ == "__main__":
    main()

#EOF
