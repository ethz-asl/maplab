import os

name = 'maplab'

on_rtd = os.environ.get('READTHEDOCS', None) == 'True'

if not on_rtd:  # only import and set the theme if we're building docs locally
    import sphinx_rtd_theme
    html_theme = 'sphinx_rtd_theme'
    html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

html_logo = "logo.png"

#source_parsers = {
#    '.md': 'recommonmark.parser.CommonMarkParser',
#}

extensions = [
    'breathe', 'exhale', 'sphinx.ext.autosectionlabel', 'recommonmark'
]

project = name
master_doc = 'index'

html_theme_options = {'logo_only': True}

# Setup the breathe extension
breathe_projects = {"project": "./doxyoutput/xml"}
breathe_default_project = "project"

# Setup the exhale extension
exhale_args = {
    "verboseBuild": False,
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    "rootFileTitle": "Library API",
    "doxygenStripFromPath": "..",
    "createTreeView": True,
    "exhaleExecutesDoxygen": True,
    "exhaleUseDoxyfile": True,
    "pageLevelConfigMeta": ":github_url: https://github.com/ethz-asl/" + name
}

source_suffix = ['.rst', '.md']

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'
