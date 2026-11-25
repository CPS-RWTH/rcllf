#!/usr/bin/env python3
# Copyright 2026 rcllf Contributors
# Licensed under the Apache License, Version 2.0
#
# Check that no two workspace packages install a library file of the same
# name. colcon installs each package's libraries into its own
# install/<pkg>/lib, and install/setup.bash puts every one of those dirs on
# LD_LIBRARY_PATH. An executable's INSTALL_RPATH ($ORIGIN/..) is a RUNPATH,
# which LD_LIBRARY_PATH precedes, so when two packages both install
# lib<name>.so the package sourced later shadows the other one's library and
# that package's executables die at load with "symbol lookup error".
#
# A clash is two packages installing the same file name: the effective output
# file name includes the suffix, so libsim.a (STATIC) and libsim.so (SHARED,
# MODULE, or no type given: ament builds shared) do not clash.
#
# Scans every CMakeLists.txt under packages/ (build/, install/, log/ and
# third_party/ excluded). Each file is split into its commands with a small
# CMake lexer (comments, quoted and bracket arguments are not commands), and
# the commands run in source order: project(), set() (plain and CACHE forms)
# and unset() update the variables, add_library / ament_auto_add_library record
# a target under the name in effect at that command (IMPORTED, ALIAS,
# INTERFACE and OBJECT targets produce no file and are skipped),
# add_subdirectory() scans the child with a copy of the current variables and
# the same target registry, and set_target_properties / set_property(TARGET)
# apply OUTPUT_NAME, LIBRARY_OUTPUT_NAME (shared and module; wins over
# OUTPUT_NAME) and ARCHIVE_OUTPUT_NAME (static). Names are grouped by package
# (nearest package.xml). Exit 1 on any file name installed by more than one
# package, 0 otherwise.
#
# This is a heuristic lint, not a CMake evaluator. Constructs it does NOT
# model (a clash hidden behind one of them is missed, or a name is misread):
#   - set(... PARENT_SCOPE): ignored, the caller's variables do not change;
#   - set(... CACHE ... FORCE): treated like a non-FORCE cache set (it does not
#     overwrite a variable already set);
#   - generator expressions ($<...>), $ENV{...}, $CACHE{...} and nested ${${x}}
#     references: left unresolved in the name;
#   - if()/else()/foreach()/while()/macro()/function() bodies: every branch and
#     body runs once, in source order, with no arguments bound;
#   - include(), find_package() and other files' variables or targets (only
#     add_subdirectory is followed), list(), string() and option();
#   - target properties PREFIX, SUFFIX, <CONFIG>_OUTPUT_NAME, *_POSTFIX and
#     VERSION/SOVERSION symlinks; BUILD_SHARED_LIBS (no type means shared);
#   - install(FILES/PROGRAMS/DIRECTORY ...) of prebuilt libraries and
#     install(TARGETS) with a renamed destination file;
#   - set_target_properties values given as a quoted list.
#
# Usage: scripts/check_library_names.py [ROOT]   (ROOT defaults to packages/)

import os
import re
import sys

SKIP_DIRS = {'build', 'install', 'log', 'third_party', '.git'}
NO_FILE = {'IMPORTED', 'ALIAS', 'INTERFACE', 'OBJECT'}
LIB_TYPES = {'STATIC', 'SHARED', 'MODULE'}
NAME_PROPS = {'OUTPUT_NAME', 'LIBRARY_OUTPUT_NAME', 'ARCHIVE_OUTPUT_NAME'}
_BRACKET_OPEN = re.compile(r'\[(=*)\[')
_COMMAND = re.compile(r'([A-Za-z_][A-Za-z0-9_]*)[ \t]*\(')


def _bracket_end(text, i):
    """If a bracket argument or comment opens at text[i], return the index past its close."""
    m = _BRACKET_OPEN.match(text, i)
    if not m:
        return None
    close = text.find(']' + m.group(1) + ']', m.end())
    return len(text) if close < 0 else close + len(m.group(1)) + 2


def _quoted_end(text, i):
    """Return the index past the quoted argument opening at text[i] (escapes honoured)."""
    i += 1
    while i < len(text) and text[i] != '"':
        i += 2 if text[i] == '\\' else 1
    return i + 1


def _skip_comment(text, i):
    """Return the index past the comment opening at text[i] ('#')."""
    end = _bracket_end(text, i + 1)
    if end is not None:
        return end
    nl = text.find('\n', i)
    return len(text) if nl < 0 else nl


def commands(text):
    """
    Return [(lowercased command name, [argument tokens])] in source order.

    A token is (text, quoted): quoted for "..." and [[...]] arguments, whose
    content is never a command. Unquoted arguments containing nested parens
    stay one token per word, as CMake splits them.
    """
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c == '#':
            i = _skip_comment(text, i)
            continue
        m = _COMMAND.match(text, i)
        if not m or (i and (text[i - 1].isalnum() or text[i - 1] == '_')):
            i += 1
            continue
        i, args, depth = m.end(), [], 1
        while i < n and depth:
            c = text[i]
            if c == '#':
                i = _skip_comment(text, i)
            elif c == '"':
                end = _quoted_end(text, i)
                args.append((text[i + 1:end - 1], True))
                i = end
            elif c == '[' and _bracket_end(text, i) is not None:
                end = _bracket_end(text, i)
                inner = _BRACKET_OPEN.match(text, i)
                args.append((text[inner.end():end - len(inner.group(1)) - 2], True))
                i = end
            elif c == '(':
                depth += 1
                i += 1
            elif c == ')':
                depth -= 1
                i += 1
            elif c.isspace():
                i += 1
            else:
                j = i
                while j < n and not text[j].isspace() and text[j] not in '()#"':
                    j += 1
                args.append((text[i:j], False))
                i = j
        out.append((m.group(1).lower(), args))
    return out


def _resolve(token, variables):
    text, quoted = token
    return re.sub(r'\$\{(\w+)\}', lambda m: variables.get(m.group(1), m.group(0)), text)


class _Lib:
    def __init__(self, target, kind):
        self.target, self.kind, self.props = target, kind, {}

    def file_name(self):
        """Return the installed file name: lib<stem>.a (STATIC) or lib<stem>.so."""
        if self.kind == 'STATIC':
            key, suffix = 'ARCHIVE_OUTPUT_NAME', '.a'
        else:  # SHARED, MODULE, or the default (ament builds shared)
            key, suffix = 'LIBRARY_OUTPUT_NAME', '.so'
        stem = self.props.get(key) or self.props.get('OUTPUT_NAME') or self.target
        return f'lib{stem}{suffix}'


def _set_props(libs, targets, pairs):
    for prop, value in pairs:
        if prop in NAME_PROPS:
            for tgt in targets:
                if tgt in libs:
                    libs[tgt].props[prop] = value


def _set_value(toks):
    """
    Return (variable, value) for a set() that assigns in the current scope, else None.

    set(VAR v...) and set(VAR v... CACHE TYPE doc [FORCE]) assign (a list
    joins with ';'); set(VAR v... PARENT_SCOPE) does not change this scope.
    """
    if len(toks) < 2 or 'PARENT_SCOPE' in toks[1:]:
        return None
    values = toks[1:toks.index('CACHE')] if 'CACHE' in toks[1:] else toks[1:]
    return toks[0], ';'.join(values)


def library_names(cmake_path, inherited=None, visited=None, libs=None):
    """
    Return ({output file name: target}, variables) for one CMakeLists.txt.

    `inherited` holds the variables in effect where the file is entered
    (add_subdirectory passes a copy of the caller's). Subdirectories it adds
    are scanned in place, recorded in `visited` (a set of paths), and share
    the caller's target registry `libs`, since CMake target names are global:
    a parent may set properties on a target a subdirectory defined. Output
    names are computed once the outermost file has run. The returned
    variables are those at the end of the file.
    """
    with open(cmake_path, encoding='utf-8') as f:
        cmds = commands(f.read())
    if visited is not None:
        visited.add(os.path.abspath(cmake_path))
    outermost = libs is None
    libs = {} if outermost else libs
    variables = dict(inherited or {})
    for name, raw in cmds:
        toks = [_resolve(t, variables) for t in raw]
        if name == 'project' and toks:
            variables['PROJECT_NAME'] = toks[0]
        elif name == 'set' and _set_value(toks):
            var, value = _set_value(toks)
            if 'CACHE' not in toks[1:] or var not in variables:
                variables[var] = value
        elif name in ('set', 'unset') and len(toks) == 1:
            variables.pop(toks[0], None)
        elif name in ('add_library', 'ament_auto_add_library') and toks:
            upper = [t.upper() for t in toks[1:]]
            if not NO_FILE.intersection(upper):
                kind = next((t for t in upper if t in LIB_TYPES), 'SHARED')
                libs[toks[0]] = _Lib(toks[0], kind)
        elif name == 'set_target_properties' and 'PROPERTIES' in toks:
            i = toks.index('PROPERTIES')
            rest = toks[i + 1:]
            _set_props(libs, toks[:i], zip(rest[0::2], rest[1::2]))
        elif name == 'set_property' and toks[:1] == ['TARGET'] and 'PROPERTY' in toks:
            # set_property(TARGET t1 [t2 ...] [APPEND|APPEND_STRING] PROPERTY name value)
            i = toks.index('PROPERTY')
            tgts = [t for t in toks[1:i] if t not in ('APPEND', 'APPEND_STRING')]
            if len(toks) > i + 2:
                _set_props(libs, tgts, [(toks[i + 1], toks[i + 2])])
        elif name == 'add_subdirectory' and toks:
            sub = os.path.join(os.path.dirname(cmake_path), toks[0], 'CMakeLists.txt')
            if os.path.isfile(sub):
                library_names(sub, variables, visited, libs)
    names = {lib.file_name(): tgt for tgt, lib in libs.items()} if outermost else {}
    return names, variables


def package_of(path, root):
    d = os.path.dirname(path)
    while True:
        if os.path.isfile(os.path.join(d, 'package.xml')):
            return os.path.basename(d)
        if os.path.abspath(d) == os.path.abspath(root) or d in ('', os.sep):
            return os.path.basename(os.path.dirname(path))
        d = os.path.dirname(d)


def scan(root):
    """Return {library file name: {package: CMakeLists path}}."""
    seen = {}
    visited = set()
    scope = {}  # dir -> variables at the end of the nearest scanned CMakeLists above it
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS)
        parent = os.path.dirname(dirpath)
        inherited = {} if 'package.xml' in filenames else scope.get(parent, {})
        scope[dirpath] = inherited
        path = os.path.join(dirpath, 'CMakeLists.txt')
        # Files reached through add_subdirectory were scanned with their caller.
        if 'CMakeLists.txt' not in filenames or os.path.abspath(path) in visited:
            continue
        pkg = package_of(path, root)
        names, scope[dirpath] = library_names(path, inherited, visited)
        for name in names:
            seen.setdefault(name, {})[pkg] = path
    return seen


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    root = argv[1] if len(argv) > 1 else os.path.join(here, '..', 'packages')
    seen = scan(root)
    clashes = {n: p for n, p in sorted(seen.items()) if len(p) > 1}
    for name, pkgs in clashes.items():
        where = ', '.join(f'{pkg} ({os.path.relpath(path)})' for pkg, path in sorted(pkgs.items()))
        print(f'DUPLICATE library {name}: built by {len(pkgs)} packages: {where}')
    if clashes:
        print(f'FAIL: {len(clashes)} library file name(s) installed by more than one package. '
              'The package sourced later shadows the other via LD_LIBRARY_PATH; '
              'prefix the target with its package name.')
        return 1
    print(f'OK: {len(seen)} library file names, each built by one package')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
