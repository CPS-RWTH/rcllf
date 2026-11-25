# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
"""Parsing cases of scripts/check_library_names.py."""

import os
import subprocess
import sys

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'check_library_names.py')


def tree(tmp_path, files):
    for rel, text in files.items():
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
    return subprocess.run([sys.executable, SCRIPT, str(tmp_path)], capture_output=True,
                          text=True)


def test_repo_has_no_clash():
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    assert out.returncode == 0, out.stdout


def test_plain_clash(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt': 'project(a)\nadd_library(sim SHARED x.cpp)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert out.returncode == 1
    assert 'DUPLICATE library libsim.so: built by 2 packages: a' in out.stdout


def test_nested_file_sees_parent_project_name(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt': 'project(foo)\nadd_subdirectory(sub)\n',
        'a/sub/CMakeLists.txt': 'add_library(${PROJECT_NAME}_c SHARED x.cpp)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(foo_c SHARED y.cpp)\n'})
    assert 'DUPLICATE library libfoo_c' in out.stdout, out.stdout


def test_set_property_output_name(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\nadd_library(u SHARED x.cpp)\n'
            'set_property(TARGET u PROPERTY OUTPUT_NAME sim)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert 'DUPLICATE library libsim' in out.stdout, out.stdout


def test_set_target_properties_output_name(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\nadd_library(u SHARED x.cpp)\n'
            'set_target_properties(u PROPERTIES OUTPUT_NAME sim)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert 'DUPLICATE library libsim' in out.stdout, out.stdout


def test_comments_imported_interface_ignored(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\n#[[\nadd_library(sim SHARED x.cpp)\n]]\n'
            '# add_library(sim SHARED x.cpp)\nadd_library(mujoco SHARED IMPORTED)\n'
            'add_library(iface INTERFACE)\n',
        'b/package.xml': '', 'b/CMakeLists.txt':
            'project(b)\nadd_library(sim SHARED y.cpp)\nadd_library(mujoco SHARED IMPORTED)\n'
            'add_library(iface INTERFACE)\n'})
    assert out.returncode == 0, out.stdout


# Round 2 review: commands run in source order, LIBRARY_OUTPUT_NAME
# is read, and command text inside quoted or bracket arguments is not a command.

def test_set_applies_in_source_order(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\nset(LIB sim)\nadd_library(${LIB} SHARED x.cpp)\nset(LIB other)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert 'DUPLICATE library libsim' in out.stdout, out.stdout
    assert 'libother' not in out.stdout, out.stdout


def test_set_target_properties_library_output_name(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\nadd_library(u SHARED x.cpp)\n'
            'set_target_properties(u PROPERTIES LIBRARY_OUTPUT_NAME sim)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert 'DUPLICATE library libsim' in out.stdout, out.stdout


def test_set_property_library_output_name_wins_over_output_name(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\nadd_library(u SHARED x.cpp)\n'
            'set_property(TARGET u PROPERTY LIBRARY_OUTPUT_NAME sim)\n'
            'set_property(TARGET u PROPERTY OUTPUT_NAME other)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert 'DUPLICATE library libsim' in out.stdout, out.stdout
    assert 'libother' not in out.stdout, out.stdout


def test_command_text_in_quoted_or_bracket_argument_ignored(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\nmessage(STATUS "add_library(sim SHARED x.cpp)")\n'
            'message(STATUS [=[\nadd_library(sim SHARED x.cpp)\n]=])\n'
            'message(STATUS "escaped \\" add_library(sim SHARED x.cpp)")\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert out.returncode == 0, out.stdout


# Round 3 review: a parent sets properties on a target its
# subdirectory defined; set() in its CACHE form assigns too.

def test_parent_property_on_subdirectory_target(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\nadd_subdirectory(sub)\n'
            'set_target_properties(u PROPERTIES LIBRARY_OUTPUT_NAME sim)\n',
        'a/sub/CMakeLists.txt': 'add_library(u SHARED x.cpp)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert 'DUPLICATE library libsim' in out.stdout, out.stdout


def test_set_cache_form_assigns(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt':
            'project(a)\nset(LIB sim CACHE STRING "library name")\n'
            'add_library(${LIB} SHARED x.cpp)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert 'DUPLICATE library libsim' in out.stdout, out.stdout


# Round 5: a clash is two packages installing the same file name,
# so the static/shared suffix is part of the name.

def test_static_and_shared_of_same_stem_do_not_clash(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt': 'project(a)\nadd_library(sim STATIC x.cpp)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim SHARED y.cpp)\n'})
    assert out.returncode == 0, out.stdout


def test_two_static_of_same_name_clash(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt': 'project(a)\nadd_library(sim STATIC x.cpp)\n',
        'b/package.xml': '', 'b/CMakeLists.txt':
            'project(b)\nadd_library(u STATIC y.cpp)\n'
            'set_target_properties(u PROPERTIES ARCHIVE_OUTPUT_NAME sim)\n'})
    assert 'DUPLICATE library libsim.a: built by 2 packages' in out.stdout, out.stdout


def test_module_and_untyped_install_so(tmp_path):
    out = tree(tmp_path, {
        'a/package.xml': '', 'a/CMakeLists.txt': 'project(a)\nadd_library(sim MODULE x.cpp)\n',
        'b/package.xml': '', 'b/CMakeLists.txt': 'project(b)\nadd_library(sim y.cpp)\n'})
    assert 'DUPLICATE library libsim.so' in out.stdout, out.stdout
