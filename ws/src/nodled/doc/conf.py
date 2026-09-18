# Copyright 2026 Alistair English, Emerson Knapp
# SPDX-License-Identifier: Apache-2.0
"""Sphinx configuration for the standalone rosdoc2 build."""

extensions = [
    'myst_parser',
    'nodl_docgen',
]

html_theme = 'furo'
html_static_path = ['_static']
html_css_files = ['custom.css']

# Keep Furo instead of rosdoc2's default Read the Docs theme. Furo provides a
# light/dark/auto selector and follows the system color scheme by default.
rosdoc2_settings = {
    'override_theme': False,
}

# rosdoc2 stages authored pages under user_docs/ and also copies this Sphinx
# project to its wrapper root. Ignore the duplicate root copy.
exclude_patterns = ['overview.md']
