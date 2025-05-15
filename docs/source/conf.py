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

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
]

language = 'en'
exclude_patterns = []
templates_path = ['_templates']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = 'alabaster'
#html_theme = "sphinx_rtd_theme"
html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
html_static_path = ['_static']

master_doc = 'index'

