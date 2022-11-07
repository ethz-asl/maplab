# -*- coding: utf-8 -*-
"""Sphinx configuration file."""

import time

import sphinx_rtd_theme
html_theme = 'sphinx_rtd_theme'
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

# html_theme = "alabaster"
html_logo = "logo.png"

project = "maplab"
master_doc = 'index'
author = "ASL Mapping Team"
copyright = "{}, {}".format(time.strftime("%Y"), author)
html_last_updated_fmt = "%c"
pygments_style = "sphinx"
templates_path = ["_templates"]

# Check https://holzhaus.github.io/sphinx-multiversion/master/
#               configuration.html#tag-branch-remote-whitelists
smv_tag_whitelist = None  # Include no tags
smv_branch_whitelist = r'^(master|wiki_update)$'  # Everything is on master now
smv_remote_whitelist = r'^(origin|upstream)$'  # Use origin and upstream

extensions = [
    'sphinx.ext.autosectionlabel', 'recommonmark', 'sphinx_multiversion'
]

html_theme_options = {
    'canonical_url': 'maplab.asl.ethz.ch',
    'analytics_id': 'UA-XXXXXXX-1',  #  Provided by Google in your dashboard
    'logo_only': True,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': True,
    'style_nav_header_background': 'green',
    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    # 'includehidden': True,
    # 'titles_only': False
}

html_context = {
    'display_github': True,
    'github_repo': 'maplab',
    'github_user': 'ethz-asl',
    'github_version': 'master',
    'conf_py_path': '/docs/',
}

templates_path = [
    "_templates",
]

source_suffix = ['.rst', '.md']

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'
