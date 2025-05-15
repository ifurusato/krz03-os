# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os, sys
import sphinx_rtd_theme

#sys.path.insert(0, os.path.abspath('../../'))
sys.path.insert(0, os.path.abspath('/home/pi/workspaces/workspace-krzos/krzos'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project   = 'krzos'
copyright = '©2025, Ichiro Furusato'
author    = 'Ichiro Furusato'
release   = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# mock the root-level modules so that Sphinx can process them without executing any code
autodoc_mock_imports = [
    'krzos',
    'krzosd',
]

# exclude specific files or patterns
exclude_patterns = [
    '*/irq_clock.py',
    '*/sound-template.py',
]

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.autosummary',
    'sphinx_rtd_theme',
]

language = 'en'
exclude_patterns = []
templates_path = ['_templates']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = 'alabaster'
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

master_doc = 'index'

